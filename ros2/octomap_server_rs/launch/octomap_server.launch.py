"""Launch octomap_server_rs, optionally with RViz.

    ros2 launch octomap_server_rs octomap_server.launch.py \
        cloud_in:=/camera/depth/points frame_id:=map resolution:=0.05

Every argument below maps onto a node parameter of the same name. The two a
first run usually needs are ``cloud_in`` (which topic the sensor publishes on)
and ``frame_id`` (which frame the map should be built in); the rest have
working defaults.

Launch arguments arrive as strings, so each parameter is wrapped in a
``ParameterValue`` with its real type. Without that, ``resolution:=0.05``
reaches the node as the string ``"0.05"``, the node reads no double there, and
it silently maps at the default resolution instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# (name, default, type, description)
PARAMETERS = [
    ("resolution", "0.1", float, "Smallest voxel edge length, in meters"),
    ("frame_id", "map", str, "Frame the map is built and published in"),
    ("sensor_frame_id", "", str, "Override the cloud's frame_id (empty: trust it)"),
    ("use_tf", "true", bool, "Resolve the cloud's frame into frame_id through /tf"),
    ("max_range", "-1.0", float, "Longest ray integrated; negative is unlimited"),
    ("min_range", "0.0", float, "Drop points closer than this to the sensor"),
    ("min_z", "-1e308", float, "Lowest point height kept, in the map frame"),
    ("max_z", "1e308", float, "Highest point height kept, in the map frame"),
    ("point_stride", "1", int, "Keep one point in every N"),
    ("publish_period", "1.0", float, "Seconds between map publishes"),
    ("publish_free_markers", "false", bool, "Also draw free cells (there are many)"),
    ("compress_map", "true", bool, "Prune the tree after every scan"),
]

OTHER_ARGUMENTS = [
    ("cloud_in", "/cloud_in", "Incoming sensor_msgs/PointCloud2 topic"),
    ("rviz", "false", "Start RViz with the bundled config"),
]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in PARAMETERS
    ] + [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in OTHER_ARGUMENTS
    ]

    server = Node(
        package="octomap_server_rs",
        executable="octomap_server_rs",
        name="octomap_server",
        output="screen",
        parameters=[
            {
                name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                for name, _, kind, _ in PARAMETERS
            }
        ],
        # The node subscribes to "cloud_in"; this points that at the real topic
        # without the node needing to know its name.
        remappings=[("cloud_in", LaunchConfiguration("cloud_in"))],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("octomap_server_rs"), "rviz", "octomap.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(declarations + [server, rviz])
