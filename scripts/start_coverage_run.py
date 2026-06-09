#!/usr/bin/env python3

import sys
import time
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Time as RosTimeMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath, NavigateToPose, UndockRobot
from open_mower_next.action import DockRobotNearest
from open_mower_next.srv import AreaCoverage


class CoverageRunStarter(Node):
    def __init__(self):
        super().__init__("start_coverage_run")

        self.declare_parameter("area_id", "operation-1")
        self.declare_parameter("with_exclusions", True)
        self.declare_parameter("headland_loops", 3)
        self.declare_parameter("swath_angle", 0)
        self.declare_parameter("undock_action", "/undock_robot")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("dock_action", "/dock_robot_nearest")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("dock_type", "")
        self.declare_parameter("max_undocking_time", 30.0)
        self.declare_parameter("segment_gap_threshold", 0.5)
        self.declare_parameter("follow_path_chunk_length", 2.0)
        self.declare_parameter("follow_path_min_chunk_length", 0.4)
        self.declare_parameter("follow_path_completion_tolerance", 0.2)
        self.declare_parameter("follow_path_completion_arm_distance", 0.6)
        self.declare_parameter("transition_settle_sec", 1.0)
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("wait_timeout_sec", 30.0)

        self.area_coverage_client = self.create_client(AreaCoverage, "/area_coverage")
        self.undock_client = ActionClient(
            self, UndockRobot, self.get_parameter("undock_action").value
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, self.get_parameter("navigate_action").value
        )
        self.follow_path_client = ActionClient(
            self, FollowPath, self.get_parameter("follow_path_action").value
        )
        self.dock_client = ActionClient(
            self, DockRobotNearest, self.get_parameter("dock_action").value
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    @staticmethod
    def latest_tf_stamp():
        return RosTimeMsg()

    def wait_for_interfaces(self) -> bool:
        timeout = float(self.get_parameter("wait_timeout_sec").value)

        self.get_logger().info("Waiting for /area_coverage service")
        if not self.area_coverage_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/area_coverage service is not available")
            return False

        self.get_logger().info(
            f"Waiting for {self.get_parameter('undock_action').value} action"
        )
        if not self.undock_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("Undock action server is not available")
            return False

        self.get_logger().info(
            f"Waiting for {self.get_parameter('navigate_action').value} action"
        )
        if not self.navigate_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("NavigateToPose action server is not available")
            return False

        self.get_logger().info(
            f"Waiting for {self.get_parameter('follow_path_action').value} action"
        )
        if not self.follow_path_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("FollowPath action server is not available")
            return False

        self.get_logger().info(
            f"Waiting for {self.get_parameter('dock_action').value} action"
        )
        if not self.dock_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("DockRobotNearest action server is not available")
            return False

        return True

    def run_sequence(self) -> int:
        if not self.wait_for_interfaces():
            return 1

        if not self.undock():
            return 2

        coverage_path = self.plan_coverage()
        if coverage_path is None:
            return 3

        segments = self.split_coverage_segments(coverage_path)
        if not segments:
            self.get_logger().error("Coverage path did not contain any runnable segments")
            return 3

        for segment_index, segment in enumerate(segments):
            self.get_logger().info(
                "Starting coverage segment %d/%d with %d poses"
                % (segment_index + 1, len(segments), len(segment.poses))
            )
            if not self.navigate_to_pose(
                segment.poses[0], f"coverage segment {segment_index + 1} start"
            ):
                return 4

            chunks = self.split_follow_path_chunks(segment)
            self.get_logger().info(
                "Split coverage segment %d into %d FollowPath chunk(s)"
                % (segment_index + 1, len(chunks))
            )
            for chunk_index, chunk in enumerate(chunks):
                label = (
                    "coverage segment %d chunk %d/%d"
                    % (segment_index + 1, chunk_index + 1, len(chunks))
                )
                if not self.follow_path(chunk, label):
                    return 5

        if not self.dock_nearest():
            return 6

        self.get_logger().info("Coverage run sequence completed")
        return 0

    @staticmethod
    def distance_between_poses(a, b) -> float:
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        return (dx * dx + dy * dy) ** 0.5

    def split_coverage_segments(self, path: Path):
        threshold = float(self.get_parameter("segment_gap_threshold").value)
        segments = []
        current = Path()
        current.header = path.header

        for pose in path.poses:
            if (
                current.poses
                and self.distance_between_poses(current.poses[-1], pose) > threshold
            ):
                segments.append(current)
                current = Path()
                current.header = path.header
            current.poses.append(deepcopy(pose))

        if current.poses:
            segments.append(current)

        self.get_logger().info(
            "Split coverage path into %d object segment(s) using %.3f m gap threshold"
            % (len(segments), threshold)
        )
        return segments

    def split_follow_path_chunks(self, path: Path):
        if len(path.poses) < 2:
            return [path]

        chunk_length = float(self.get_parameter("follow_path_chunk_length").value)
        min_chunk_length = float(self.get_parameter("follow_path_min_chunk_length").value)
        if chunk_length <= 0.0:
            return [path]

        chunks = []
        current = Path()
        current.header = path.header
        current.poses.append(deepcopy(path.poses[0]))
        current_length = 0.0

        for pose in path.poses[1:]:
            previous = current.poses[-1]
            current.poses.append(deepcopy(pose))
            current_length += self.distance_between_poses(previous, pose)

            if current_length >= chunk_length:
                chunks.append(current)
                current = Path()
                current.header = path.header
                current.poses.append(deepcopy(pose))
                current_length = 0.0

        if len(current.poses) > 1:
            if chunks and current_length < min_chunk_length:
                chunks[-1].poses.extend(deepcopy(current.poses[1:]))
            else:
                chunks.append(current)

        return chunks if chunks else [path]

    def dock_nearest(self) -> bool:
        goal = DockRobotNearest.Goal()

        self.get_logger().info("Sending DockRobotNearest goal")
        future = self.dock_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DockRobotNearest goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.code != result.CODE_SUCCESS:
            self.get_logger().error(
                f"Docking failed: code={result.code} message='{result.message}'"
            )
            return False

        self.get_logger().info("Docking completed")
        self.settle_transition("dock")
        return True

    def follow_path(self, path: Path, label: str) -> bool:
        timeout = float(self.get_parameter("wait_timeout_sec").value)
        if not self.follow_path_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("FollowPath action server is not available")
            return False

        stamp = self.latest_tf_stamp()
        path.header.stamp = stamp
        for pose in path.poses:
            pose.header.stamp = stamp

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = str(self.get_parameter("controller_id").value)
        goal.goal_checker_id = ""
        goal.progress_checker_id = ""

        self.get_logger().info(f"Sending FollowPath goal for {label}")
        future = self.follow_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"FollowPath goal for {label} was rejected")
            return False

        result_future = goal_handle.get_result_async()
        completion_armed = False
        while rclpy.ok() and not result_future.done():
            distance_to_end = self.distance_to_path_end(path)
            if distance_to_end is not None:
                if distance_to_end >= float(
                    self.get_parameter("follow_path_completion_arm_distance").value
                ):
                    completion_armed = True

                if completion_armed and distance_to_end <= float(
                    self.get_parameter("follow_path_completion_tolerance").value
                ):
                    self.get_logger().info(
                        "Robot reached end of %s "
                        "(distance_to_end=%.3f m); switching to next segment"
                        % (label, distance_to_end)
                    )
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future)
                    self.settle_transition(label)
                    return True

            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result().result
        if result.error_code != result.NONE:
            distance_to_end = self.distance_to_path_end(path)
            if completion_armed and distance_to_end is not None and distance_to_end <= float(
                self.get_parameter("follow_path_completion_tolerance").value
            ):
                self.get_logger().info(
                    "FollowPath for %s returned code=%d after reaching path end "
                    "(distance_to_end=%.3f m); switching to next segment"
                    % (label, result.error_code, distance_to_end)
                )
                self.settle_transition(label)
                return True

            self.get_logger().error(
                f"FollowPath for {label} failed: code={result.error_code} "
                f"message='{result.error_msg}'"
            )
            return False

        self.get_logger().info(f"Completed {label}")
        self.settle_transition(label)
        return True

    def distance_to_path_end(self, path: Path):
        if not path.poses:
            return None

        target_frame = path.header.frame_id or path.poses[-1].header.frame_id
        robot_frame = str(self.get_parameter("robot_frame").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return None

        end_position = path.poses[-1].pose.position
        dx = transform.transform.translation.x - end_position.x
        dy = transform.transform.translation.y - end_position.y
        return (dx * dx + dy * dy) ** 0.5

    def settle_transition(self, label: str):
        delay = float(self.get_parameter("transition_settle_sec").value)
        if delay <= 0.0:
            return

        self.get_logger().info("Settling transition after %s for %.2f s" % (label, delay))
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def navigate_to_pose(self, pose, label: str) -> bool:
        start_pose = deepcopy(pose)
        start_pose.header.stamp = self.latest_tf_stamp()

        if not self.navigate_to_start(start_pose, label):
            return False
        return True

    def undock(self) -> bool:
        goal = UndockRobot.Goal()
        goal.dock_type = str(self.get_parameter("dock_type").value)
        goal.max_undocking_time = float(self.get_parameter("max_undocking_time").value)

        self.get_logger().info("Sending undock goal")
        future = self.undock_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Undock goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(
                f"Undock failed: code={result.error_code} message='{result.error_msg}'"
            )
            return False

        self.get_logger().info("Undock completed")
        self.settle_transition("undock")
        return True

    def plan_coverage(self):
        request = AreaCoverage.Request()
        request.area_id = str(self.get_parameter("area_id").value)
        request.with_exclusions = bool(self.get_parameter("with_exclusions").value)
        request.headland_loops = int(self.get_parameter("headland_loops").value)
        request.swath_angle = int(self.get_parameter("swath_angle").value)

        self.get_logger().info(
            f"Planning coverage for area '{request.area_id}' "
            f"(exclusions={request.with_exclusions}, "
            f"headland_loops={request.headland_loops}, swath_angle={request.swath_angle})"
        )
        future = self.area_coverage_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is None:
            self.get_logger().error("Coverage planning service returned no response")
            return None
        if response.code != response.CODE_SUCCESS:
            self.get_logger().error(
                f"Coverage planning failed: code={response.code} message='{response.message}'"
            )
            return None
        if not response.path.poses:
            self.get_logger().error("Coverage planner returned an empty path")
            return None

        self.get_logger().info(
            "Coverage path contains %d poses; start pose is x=%.3f y=%.3f frame=%s"
            % (
                len(response.path.poses),
                response.path.poses[0].pose.position.x,
                response.path.poses[0].pose.position.y,
                response.path.header.frame_id,
            )
        )
        return response.path

    def navigate_to_start(self, start_pose, label: str = "coverage start") -> bool:
        timeout = float(self.get_parameter("wait_timeout_sec").value)
        if not self.navigate_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("NavigateToPose action server is not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = start_pose
        goal.behavior_tree = ""

        self.get_logger().info(f"Sending NavigateToPose goal to {label}")
        future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != result.NONE:
            self.get_logger().error(
                f"NavigateToPose failed: code={result.error_code} message='{result.error_msg}'"
            )
            return False

        self.get_logger().info(f"Robot reached {label}")
        self.settle_transition(label)
        return True


def main():
    rclpy.init()
    node = CoverageRunStarter()
    exit_code = 1
    try:
        exit_code = node.run_sequence()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
