import json
import math
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import webots_ros2_driver.utils as webots_utils
import webots_ros2_driver.webots_controller as webots_controller_module
import webots_ros2_driver.webots_launcher as webots_launcher_module
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher, WebotsLauncher


WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
DOCK_CONTACT_OFFSET_X = 0.72
ROBOT_CHARGING_PORT_OFFSET_X = 0.45


def configure_windows_webots_controller_ip():
    webots_home = os.environ.get("ROS2_WEBOTS_HOME") or os.environ.get("WEBOTS_HOME", "")
    windows_webots_home = webots_home or "/mnt/c/Program Files/Webots"

    if not webots_utils.is_wsl() or not windows_webots_home.startswith("/mnt/"):
        return

    host_ip = os.environ.get("OPEN_MOWER_WEBOTS_HOST_IP")
    if not host_ip:
        return

    def controller_ip_address():
        return host_ip

    def controller_url_prefix(port="1234"):
        return f"tcp://{host_ip}:{port}/"

    webots_controller_module.controller_ip_address = controller_ip_address
    webots_launcher_module.controller_url_prefix = controller_url_prefix


def geodetic_to_ecef(lat, lon, altitude=0.0):
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    radius = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + altitude) * cos_lat * cos_lon,
        (radius + altitude) * cos_lat * sin_lon,
        (radius * (1 - WGS84_E2) + altitude) * sin_lat,
    )


def lon_lat_to_map(point, datum_lat, datum_lon):
    lon = math.radians(point[0])
    lat = math.radians(point[1])
    lat0 = math.radians(datum_lat)
    lon0 = math.radians(datum_lon)

    x, y, z = geodetic_to_ecef(lat, lon)
    origin_x, origin_y, origin_z = geodetic_to_ecef(lat0, lon0)
    dx = x - origin_x
    dy = y - origin_y
    dz = z - origin_z

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    return (east, north)


def format_float(value):
    return f"{value:.6f}".rstrip("0").rstrip(".")


def polygon_to_shape(feature_id, points, color, z):
    point_lines = "\n".join(f"          {format_float(x)} {format_float(y)} {format_float(z)}" for x, y in points)
    indexes = " ".join(str(i) for i in range(len(points)))
    reverse_indexes = " ".join(str(i) for i in reversed(range(len(points))))
    return f"""Solid {{
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {color}
        roughness 1
        metalness 0
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{
          point [
{point_lines}
          ]
        }}
        coordIndex [
          {indexes} -1
          {reverse_indexes} -1
        ]
      }}
    }}
  ]
  name "{feature_id}"
}}"""


def dock_vector_shape(feature_id, approach_point, edge):
    return f"""Solid {{
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0.05 0.15 1
          emissiveColor 0.02 0.06 0.35
        }}
      }}
      geometry IndexedLineSet {{
        coord Coordinate {{
          point [
            {format_float(approach_point[0])} {format_float(approach_point[1])} 0.08
            {format_float(edge[0])} {format_float(edge[1])} 0.22
          ]
        }}
        coordIndex [
          0 1 -1
        ]
      }}
    }}
    Transform {{
      translation {format_float(approach_point[0])} {format_float(approach_point[1])} 0.08
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.95 0.95 0.05
            roughness 0.7
            metalness 0
          }}
          geometry Sphere {{
            radius 0.06
            subdivision 2
          }}
        }}
      ]
    }}
    Transform {{
      translation {format_float(edge[0])} {format_float(edge[1])} 0.12
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.05 0.15 1
            roughness 0.7
            metalness 0
          }}
          geometry Cylinder {{
            radius 0.025
            height 0.24
          }}
        }}
      ]
    }}
    Transform {{
      translation {format_float(edge[0])} {format_float(edge[1])} 0.27
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.05 0.15 1
            roughness 0.7
            metalness 0
          }}
          geometry Sphere {{
            radius 0.09
            subdivision 2
          }}
        }}
      ]
    }}
  ]
  name "{feature_id}_vector"
}}"""


def load_sim_map(share_directory):
    map_path = os.environ.get("OM_MAP_PATH")
    datum_lat = float(os.environ.get("OM_DATUM_LAT"))
    datum_lon = float(os.environ.get("OM_DATUM_LONG"))

    if not map_path:
        return None

    with open(map_path, "r", encoding="utf-8") as map_file:
        data = json.load(map_file)

    polygons = []
    dock_pose = None
    dock_vector = None
    all_points = []

    for index, feature in enumerate(data.get("features", []), start=1):
        geometry = feature.get("geometry", {})
        properties = feature.get("properties", {})
        geometry_type = geometry.get("type")
        feature_type = properties.get("type", "")

        if geometry_type == "Polygon":
            ring = geometry.get("coordinates", [[]])[0]
            points = [lon_lat_to_map(point, datum_lat, datum_lon) for point in ring]
            if len(points) > 1 and points[0] == points[-1]:
                points = points[:-1]
            if len(points) < 3:
                continue
            all_points.extend(points)
            polygons.append(
                {
                    "id": properties.get("id") or f"area_{index}",
                    "type": feature_type,
                    "points": points,
                }
            )
            continue

        if geometry_type == "LineString" and feature_type == "docking_station":
            coordinates = geometry.get("coordinates", [])
            if len(coordinates) >= 2:
                start = lon_lat_to_map(coordinates[0], datum_lat, datum_lon)
                end = lon_lat_to_map(coordinates[1], datum_lat, datum_lon)
                yaw = math.atan2(end[1] - start[1], end[0] - start[0])
                dock_pose = (start[0], start[1], yaw)
                dock_vector = (start, end)
                all_points.extend([start, end])

    if not all_points:
        return None

    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    floor_size_x = max(20.0, max_x - min_x + 4.0)
    floor_size_y = max(20.0, max_y - min_y + 4.0)
    if dock_pose:
        start_x, start_y, dock_yaw = dock_pose
        edge_x, edge_y = dock_vector[1] if dock_vector else (start_x, start_y)
        dock_x = edge_x
        dock_y = edge_y
        contact_x = dock_x + DOCK_CONTACT_OFFSET_X * math.cos(dock_yaw)
        contact_y = dock_y + DOCK_CONTACT_OFFSET_X * math.sin(dock_yaw)
        robot_x = contact_x - ROBOT_CHARGING_PORT_OFFSET_X * math.cos(dock_yaw)
        robot_y = contact_y - ROBOT_CHARGING_PORT_OFFSET_X * math.sin(dock_yaw)
        robot_yaw = dock_yaw
    else:
        contact_x = 1.82
        contact_y = 1.5
        dock_yaw = 0.0
        dock_x = 1.5
        dock_y = 1.5
        robot_x = center_x
        robot_y = center_y
        robot_yaw = 0.0

    shapes = []
    for polygon in polygons:
        if polygon["type"] == "exclusion":
            shapes.append(polygon_to_shape(polygon["id"], polygon["points"], "0.8 0.05 0.05", 0.03))
        elif polygon["type"] == "operation":
            shapes.append(polygon_to_shape(polygon["id"], polygon["points"], "0.1 0.55 0.1", 0.004))
        else:
            shapes.append(polygon_to_shape(polygon["id"], polygon["points"], "0.75 0.75 0.75", 0.003))
    if dock_vector:
        shapes.append(dock_vector_shape("docking_station", dock_vector[0], dock_vector[1]))

    world_text = f"""#VRML_SIM R2025a utf8

EXTERNPROTO "../protos/OpenMower.proto"
EXTERNPROTO "../protos/DockingStation.proto"

WorldInfo {{
  title "OpenMowerNext Webots simulation"
  basicTimeStep 10
  coordinateSystem "ENU"
  gpsCoordinateSystem "WGS84"
  gpsReference {format_float(datum_lat)} {format_float(datum_lon)} 0
}}
Viewpoint {{
  orientation -0.330491 0.451759 0.828566 1.32217
  position {format_float(center_x - 6.0)} {format_float(center_y - 8.0)} 8
  follow "openmower"
}}
Background {{
  skyColor [
    0.45 0.62 0.85
  ]
}}
DirectionalLight {{
  direction -0.4 -0.2 -1
  intensity 1
}}
Solid {{
  translation {format_float(center_x)} {format_float(center_y)} -0.005
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.08 0.18 0.08
        roughness 1
        metalness 0
      }}
      geometry DEF FLOOR_BOX Box {{
        size {format_float(floor_size_x)} {format_float(floor_size_y)} 0.01
      }}
    }}
  ]
  name "grass_floor"
  boundingObject USE FLOOR_BOX
}}
{os.linesep.join(shapes)}
OpenMower {{
  translation {format_float(robot_x)} {format_float(robot_y)} 0.0925
  rotation 0 0 1 {format_float(robot_yaw)}
  name "openmower"
  controller "<extern>"
}}
DockingStation {{
  translation {format_float(dock_x)} {format_float(dock_y)} 0
  rotation 0 0 1 {format_float(dock_yaw)}
  name "docking_station"
}}
Robot {{
  name "Ros2Supervisor"
  controller "<extern>"
  supervisor TRUE
}}
"""

    world_file = tempfile.NamedTemporaryFile(
        "w", suffix="_openmower_map.wbt", dir=os.path.join(share_directory, "worlds"), delete=False
    )
    world_file.write(world_text)
    world_file.close()

    return {
        "world": world_file.name,
        "dock_contact_x": contact_x,
        "dock_contact_y": contact_y,
        "dock_contact_yaw": dock_yaw,
    }


def shutdown_on_driver_failure(event, context):
    if context.is_shutdown or event.returncode == 0:
        return []
    return [EmitEvent(event=Shutdown(reason="Webots driver exited unexpectedly"))]


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    package_name = "open_mower_next"
    share_directory = get_package_share_directory(package_name)
    configure_windows_webots_controller_ip()
    sim_map = load_sim_map(share_directory)

    world = LaunchConfiguration("world").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    webots_stream = LaunchConfiguration("webots_stream").perform(context).lower() in [
        "true",
        "1",
        "yes",
    ]
    webots_port = LaunchConfiguration("webots_port").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    visualize_coverage_in_webots = LaunchConfiguration("visualize_coverage_in_webots")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")

    xacro_file = os.path.join(share_directory, "description", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "use_ros2_control": "0",
            "sim_mode": "true",
        },
    ).toxml()

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config, "use_sim_time": use_sim_time}],
    )

    twist_mux_params = os.path.join(share_directory, "config", "twist_mux.yaml")
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/diff_drive_base_controller/cmd_vel")],
    )

    webots = WebotsLauncher(
        world=sim_map["world"] if sim_map else os.path.join(share_directory, "worlds", world),
        mode=mode,
        gui=gui,
        stream=webots_stream,
        port=webots_port,
    )
    webots_supervisor = Ros2SupervisorLauncher(respawn=False, port=webots_port)

    controller_params_file = os.path.join(share_directory, "config", "controllers.yaml")
    webots_robot_description = os.path.join(share_directory, "resource", "openmower_webots.urdf")
    webots_driver = WebotsController(
        robot_name="openmower",
        parameters=[
            {
                "robot_description": webots_robot_description,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": False,
            },
            controller_params_file,
        ],
        respawn=False,
        port=webots_port,
    )

    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
        parameters=[{"use_sim_time": use_sim_time}],
    )
    load_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_drive_base_controller"] + controller_manager_timeout,
        parameters=[{"use_sim_time": use_sim_time}],
    )
    load_mower_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["mower_controller"] + controller_manager_timeout,
        parameters=[{"use_sim_time": use_sim_time}],
    )
    controller_spawners = [load_joint_state_controller, load_diff_controller, load_mower_controller]

    wait_for_webots_driver = WaitForControllerConnection(
        target_driver=webots_driver,
        nodes_to_start=controller_spawners,
    )

    # Simulation helper node publishes the hardware-facing power topics.
    sim_node = Node(
        package="open_mower_next",
        executable="sim_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "docking_station_frame": "map",
                "charging_port_frame": "charging_port",
                "docking_station_contact_x": sim_map["dock_contact_x"] if sim_map else 1.82,
                "docking_station_contact_y": sim_map["dock_contact_y"] if sim_map else 1.5,
                "docking_station_contact_z": 0.06,
                "docking_station_contact_yaw": sim_map["dock_contact_yaw"] if sim_map else 0.0,
                "docking_detection_tolerance_x": 0.35,
                "docking_detection_tolerance_y": 0.25,
            }
        ],
    )

    coverage_server = Node(
        package="open_mower_next",
        executable="coverage_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    coverage_webots_visualizer = Node(
        package="open_mower_next",
        executable="coverage_webots_visualizer.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(visualize_coverage_in_webots),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_directory, "launch", "localization.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_directory, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": os.path.join(share_directory, "config", "nav2_params.yaml"),
        }.items(),
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml")
        ),
        launch_arguments={
            "address": foxglove_address,
            "port": foxglove_port,
            "include_hidden": "true",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(enable_foxglove),
    )

    return [
        webots,
        webots_supervisor,
        node_robot_state_publisher,
        twist_mux,
        webots_driver,
        wait_for_webots_driver,
        sim_node,
        coverage_server,
        coverage_webots_visualizer,
        localization,
        nav2,
        foxglove_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots_driver,
                on_exit=shutdown_on_driver_failure,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="openmower.wbt"),
            DeclareLaunchArgument("mode", default_value="realtime"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("webots_stream", default_value="false"),
            DeclareLaunchArgument("webots_port", default_value="1234"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("visualize_coverage_in_webots", default_value="true"),
            DeclareLaunchArgument("enable_foxglove", default_value="false"),
            DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            OpaqueFunction(function=launch_setup),
        ]
    )
