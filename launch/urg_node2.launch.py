from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    urg_node2_nl_pkg = get_package_share_directory('urg_node2_nl')

    urg_node2_nl = Node(
        package='urg_node2_nl',
        executable='urg_node2_nl_node',
        name=LaunchConfiguration('node_name'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[PathJoinSubstitution([urg_node2_nl_pkg, "config", "params_serial.yaml"])],
        namespace='',
        output='screen',
    )

    # parameters
    # node_name       : node name: (default)"urg_node2"
    # scan_topic_name : topic name: (default)"scan"
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='urg_node2_nl'),
        DeclareLaunchArgument('scan_topic_name', default_value='scan'),
        urg_node2_nl,
    ])

