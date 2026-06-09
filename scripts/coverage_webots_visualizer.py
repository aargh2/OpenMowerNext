#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from webots_ros2_msgs.srv import SpawnNodeFromString


class CoverageWebotsVisualizer(Node):
    def __init__(self):
        super().__init__("coverage_webots_visualizer")

        self._node_name = self.declare_parameter(
            "webots_node_name", "coverage_path_visualization"
        ).value
        self._z_offset = float(self.declare_parameter("z_offset", 0.09).value)
        self._max_points = int(self.declare_parameter("max_points", 1200).value)
        self._mainland_color = self.declare_parameter("mainland_color", [0.05, 0.45, 1.0]).value
        self._headland_color = self.declare_parameter("headland_color", [0.0, 1.0, 0.25]).value
        self._service_name = self.declare_parameter(
            "spawn_service", "/Ros2Supervisor/spawn_node_from_string"
        ).value
        self._remove_topic = self.declare_parameter(
            "remove_topic", "/Ros2Supervisor/remove_node"
        ).value

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self._spawn_client = self.create_client(SpawnNodeFromString, self._service_name)
        self._remove_pub = self.create_publisher(String, self._remove_topic, 10)
        self._pending_markers = None
        self._spawn_in_progress = False

        self.create_subscription(
            MarkerArray, "/coverage/visualization", self._markers_callback, path_qos
        )
        self.create_timer(0.5, self._flush_pending_markers)

    def _markers_callback(self, msg: MarkerArray):
        segments = self._extract_segments(msg)
        if not segments:
            self.get_logger().warn("Ignoring coverage visualization without swath segments")
            return
        self._pending_markers = msg
        self.get_logger().info(
            f"Received coverage visualization with {len(segments)} swath segments"
        )

    def _flush_pending_markers(self):
        if self._pending_markers is None or self._spawn_in_progress:
            return
        if not self._spawn_client.service_is_ready():
            self._spawn_client.wait_for_service(timeout_sec=0.1)
            return

        markers = self._pending_markers
        self._pending_markers = None
        self._spawn_in_progress = True

        remove_msg = String()
        remove_msg.data = self._node_name
        self._remove_pub.publish(remove_msg)

        request = SpawnNodeFromString.Request()
        request.data = self._markers_to_webots_node(markers)
        future = self._spawn_client.call_async(request)
        future.add_done_callback(self._spawn_done)

    def _spawn_done(self, future):
        self._spawn_in_progress = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to spawn Webots coverage path: {exc}")
            return

        if response.success:
            self.get_logger().info("Spawned coverage path visualization in Webots")
        else:
            self.get_logger().warn("Webots supervisor rejected coverage path visualization")

    def _markers_to_webots_node(self, markers: MarkerArray) -> str:
        grouped_segments = {
            "headland": self._decimate_segments(
                self._extract_segments(markers, ("headland", "headland_connector"))
            ),
            "mainland": self._decimate_segments(self._extract_segments(markers, "mainland")),
        }
        all_segments = grouped_segments["headland"] + grouped_segments["mainland"]
        start = all_segments[0][0]
        end = all_segments[-1][-1]
        shapes = "\n".join(
            self._segments_to_shape(grouped_segments["headland"], self._headland_color)
            + self._segments_to_shape(grouped_segments["mainland"], self._mainland_color)
        )

        return f"""Solid {{
  children [
{shapes}
    Transform {{
      translation {start.x:.4f} {start.y:.4f} {self._z_offset + 0.03:.4f}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.0 1.0 0.2
            emissiveColor 0.0 0.5 0.1
            roughness 1
            metalness 0
          }}
          geometry Sphere {{
            radius 0.08
          }}
        }}
      ]
    }}
    Transform {{
      translation {end.x:.4f} {end.y:.4f} {self._z_offset + 0.03:.4f}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 1.0 0.1 0.1
            emissiveColor 0.5 0.0 0.0
            roughness 1
            metalness 0
          }}
          geometry Sphere {{
            radius 0.08
          }}
        }}
      ]
    }}
  ]
  name "{self._node_name}"
}}"""

    def _segments_to_shape(self, segments: List[List], color_values: List[float]) -> List[str]:
        if not segments:
            return []

        color = " ".join(f"{float(component):.3f}" for component in color_values[:3])
        all_points = [point for segment in segments for point in segment]
        point_lines = "\n".join(
            f"            {point.x:.4f} {point.y:.4f} {self._z_offset:.4f}"
            for point in all_points
        )
        coord_index_parts = []
        point_index = 0
        for segment in segments:
            coord_index_parts.extend(
                str(index) for index in range(point_index, point_index + len(segment))
            )
            coord_index_parts.append("-1")
            point_index += len(segment)
        coord_index = " ".join(coord_index_parts)

        return [
            f"""    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor {color}
          emissiveColor {color}
        }}
      }}
      geometry IndexedLineSet {{
        coord Coordinate {{
          point [
{point_lines}
          ]
        }}
        coordIndex [
          {coord_index}
        ]
      }}
    }}"""
        ]

    def _extract_segments(self, markers: MarkerArray, namespace=None) -> List[List]:
        segments = []
        if isinstance(namespace, str):
            namespaces = {namespace}
        elif namespace is None:
            namespaces = {"headland", "headland_connector", "mainland"}
        else:
            namespaces = set(namespace)

        for marker in markers.markers:
            if marker.action != Marker.ADD or marker.type != Marker.LINE_STRIP:
                continue
            if marker.ns not in namespaces:
                continue
            if len(marker.points) < 2:
                continue
            segments.append(list(marker.points))
        return segments

    def _decimate_segments(self, segments: List[List]) -> List[List]:
        point_count = sum(len(segment) for segment in segments)
        if point_count <= self._max_points:
            return segments

        stride = math.ceil(point_count / self._max_points)
        decimated_segments = []
        for segment in segments:
            decimated = segment[::stride]
            if decimated[-1] is not segment[-1]:
                decimated.append(segment[-1])
            if len(decimated) >= 2:
                decimated_segments.append(decimated)
        return decimated_segments


def main():
    rclpy.init()
    node = CoverageWebotsVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
