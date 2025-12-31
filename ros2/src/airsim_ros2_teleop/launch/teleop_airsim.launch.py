import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vehicle_name = LaunchConfiguration('vehicle_name')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    takeoff_on_start = LaunchConfiguration('takeoff_on_start')
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    publish_clock = LaunchConfiguration('publish_clock')
    is_vulkan = LaunchConfiguration('is_vulkan')
    enable_api_control = LaunchConfiguration('enable_api_control')
    enable_object_transforms_list = LaunchConfiguration('enable_object_transforms_list')
    output = LaunchConfiguration('output')

    airsim_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('airsim_ros_pkgs'),
            'launch',
            'airsim_node.launch.py'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('vehicle_name', default_value='Drone1'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('takeoff_on_start', default_value='true'),
        DeclareLaunchArgument('host_ip', default_value='localhost'),
        DeclareLaunchArgument('host_port', default_value='41451'),
        DeclareLaunchArgument('publish_clock', default_value='False'),
        DeclareLaunchArgument('is_vulkan', default_value='True'),
        DeclareLaunchArgument('enable_api_control', default_value='False'),
        DeclareLaunchArgument('enable_object_transforms_list', default_value='True'),
        DeclareLaunchArgument('output', default_value='screen'),

        # Core AirSim ROS node (from airsim_ros_pkgs)
        IncludeLaunchDescription(
            airsim_launch,
            launch_arguments={
                'host_ip': host_ip,
                'host_port': host_port,
                'publish_clock': publish_clock,
                'is_vulkan': is_vulkan,
                'enable_api_control': enable_api_control,
                'enable_object_transforms_list': enable_object_transforms_list,
                'output': output,
            }.items()
        ),

        # # Keyboard teleop -> publishes /cmd_vel
        Node(
            package='airsim_ros2_teleop',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[('/cmd_vel', cmd_vel_topic)],
            parameters=[{
                'host_ip': host_ip,
            }],
            prefix='xterm -e'  # opens in its own terminal so it can capture keyboard
        ),

        # Bridge /cmd_vel -> AirSim API
        Node(
            package='airsim_ros2_teleop',
            executable='airsim_twist_bridge',
            name='airsim_twist_bridge',
            output='screen',
            parameters=[{
                'vehicle_name': vehicle_name,
                'cmd_vel_topic': cmd_vel_topic,
                'takeoff_on_start': takeoff_on_start,
                'command_rate_hz': 20.0,
                'max_xy_speed': 5.0,
                'max_z_speed': 2.0,
                'max_yaw_rate': 1.5,
            }]
        ),
    ])
