#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rtcm_msgs.msg import Message as RtcmMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsNtripTest(Node):
    def __init__(self, args):
        super().__init__("hw_gps_ntrip_test")
        self.args = args
        self.fix_count = 0
        self.rtcm_count = 0
        self.gps_odom_count = 0
        self.localized_odom_count = 0
        self.rtcm_bytes = 0
        self.last_fix = None
        self.last_rtcm = None
        self.last_gps_odom = None
        self.last_localized_odom = None

        self.create_subscription(NavSatFix, args.fix_topic, self.on_fix, 10)
        self.create_subscription(RtcmMessage, args.rtcm_topic, self.on_rtcm, 10)
        self.create_subscription(Odometry, args.gps_odom_topic, self.on_gps_odom, 10)
        self.create_subscription(
            Odometry, args.localized_odom_topic, self.on_localized_odom, 10
        )

    def on_fix(self, msg):
        self.fix_count += 1
        self.last_fix = msg

    def on_rtcm(self, msg):
        self.rtcm_count += 1
        self.rtcm_bytes += len(msg.message)
        self.last_rtcm = msg

    def on_gps_odom(self, msg):
        self.gps_odom_count += 1
        self.last_gps_odom = msg

    def on_localized_odom(self, msg):
        self.localized_odom_count += 1
        self.last_localized_odom = msg

    def run(self):
        start = time.monotonic()
        deadline = start + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        elapsed = max(time.monotonic() - start, 0.001)
        self.print_report(elapsed)
        return self.exit_code()

    def print_report(self, elapsed):
        nodes = sorted(name for name, _ in self.get_node_names_and_namespaces())
        print("=== GPS / NTRIP hardware test ===")
        print(f"duration: {elapsed:.2f}s")
        print()
        print("nodes:")
        print(f"  ublox_f9p:         {present('ublox_f9p', nodes)}")
        print(f"  ntrip_client_node: {present('ntrip_client_node', nodes)}")

        ntrip_params = self.get_params(
            "/ntrip_client_node",
            ("host", "port", "mountpoint", "authenticate", "rtcm_message_package"),
        )
        if ntrip_params:
            print("ntrip parameters:")
            for key in ("host", "port", "mountpoint", "authenticate", "rtcm_message_package"):
                print(f"  {key}: {ntrip_params.get(key, '<unset>')}")
        else:
            print("ntrip parameters: unavailable")

        print()
        print("topics:")
        print(
            f"  {self.args.fix_topic}: {self.fix_count} msg "
            f"({self.fix_count / elapsed:.2f} Hz)"
        )
        print(
            f"  {self.args.rtcm_topic}: {self.rtcm_count} msg "
            f"({self.rtcm_count / elapsed:.2f} Hz), {self.rtcm_bytes} bytes"
        )
        print(
            f"  {self.args.gps_odom_topic}: {self.gps_odom_count} msg "
            f"({self.gps_odom_count / elapsed:.2f} Hz)"
        )
        print(
            f"  {self.args.localized_odom_topic}: {self.localized_odom_count} msg "
            f"({self.localized_odom_count / elapsed:.2f} Hz)"
        )

        print()
        self.print_fix()
        self.print_rtcm()
        self.print_odom("gps odom", self.last_gps_odom)
        self.print_odom("localized odom", self.last_localized_odom)

    def get_params(self, node_name, names):
        client = self.create_client(GetParameters, f"{node_name}/get_parameters")
        if not client.wait_for_service(timeout_sec=self.args.param_timeout):
            self.destroy_client(client)
            return {}

        request = GetParameters.Request()
        request.names = list(names)
        future = client.call_async(request)
        deadline = time.monotonic() + self.args.param_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_client(client)

        if not future.done() or future.result() is None:
            return {}

        result = {}
        for name, value in zip(names, future.result().values):
            result[name] = parameter_value_to_python(value)
        return result

    def print_fix(self):
        if self.last_fix is None:
            print("fix: missing")
            return

        fix = self.last_fix
        print(
            "fix: "
            f"status={fix_status_name(fix.status.status)} "
            f"service={fix.status.service} "
            f"lat={fix.latitude:.8f} lon={fix.longitude:.8f} alt={fix.altitude:.3f}"
        )
        print(
            "fix covariance: "
            f"type={fix.position_covariance_type} "
            f"x={fmt_cov(fix.position_covariance[0])} "
            f"y={fmt_cov(fix.position_covariance[4])} "
            f"z={fmt_cov(fix.position_covariance[8])}"
        )

    def print_rtcm(self):
        if self.last_rtcm is None:
            print("rtcm: missing")
            return
        print(
            "rtcm: "
            f"last_frame={self.last_rtcm.header.frame_id or '<empty>'} "
            f"last_size={len(self.last_rtcm.message)} bytes"
        )

    def print_odom(self, label, msg):
        if msg is None:
            print(f"{label}: missing")
            return
        pose = msg.pose.pose
        twist = msg.twist.twist
        print(
            f"{label}: "
            f"frame={msg.header.frame_id or '<empty>'} child={msg.child_frame_id or '<empty>'} "
            f"x={pose.position.x:.4f} y={pose.position.y:.4f} "
            f"vx={twist.linear.x:.4f} wz={twist.angular.z:.4f}"
        )

    def exit_code(self):
        if self.fix_count == 0:
            return 1
        if self.args.require_fix and self.last_fix.status.status < NavSatStatus.STATUS_FIX:
            return 2
        if self.args.require_rtcm and self.rtcm_count == 0:
            return 3
        if self.args.require_gps_odom and self.gps_odom_count == 0:
            return 4
        return 0


def parameter_value_to_python(value):
    if value.type == 1:
        return value.bool_value
    if value.type == 2:
        return value.integer_value
    if value.type == 3:
        return value.double_value
    if value.type == 4:
        return value.string_value
    return "<unset>"


def present(name, nodes):
    return "yes" if name in nodes else "no"


def fix_status_name(status):
    names = {
        NavSatStatus.STATUS_NO_FIX: "NO_FIX",
        NavSatStatus.STATUS_FIX: "FIX",
        NavSatStatus.STATUS_SBAS_FIX: "SBAS_FIX",
        NavSatStatus.STATUS_GBAS_FIX: "GBAS_FIX",
    }
    return f"{names.get(status, 'UNKNOWN')}({status})"


def fmt_cov(value):
    if math.isnan(value):
        return "nan"
    return f"{value:.6g}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check real GPS, NTRIP RTCM flow, and GPS odometry topics."
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--param-timeout", type=float, default=1.0)
    parser.add_argument("--fix-topic", default="/gps/fix")
    parser.add_argument("--rtcm-topic", default="/rtcm")
    parser.add_argument("--gps-odom-topic", default="/gps/odom")
    parser.add_argument("--localized-odom-topic", default="/odometry/gps")
    parser.add_argument("--require-fix", action="store_true")
    parser.add_argument("--require-rtcm", action="store_true", default=True)
    parser.add_argument("--no-require-rtcm", dest="require_rtcm", action="store_false")
    parser.add_argument("--require-gps-odom", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = GpsNtripTest(args)
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
