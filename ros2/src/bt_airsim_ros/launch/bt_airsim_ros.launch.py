import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    publish_clock = LaunchConfiguration('publish_clock')
    is_vulkan = LaunchConfiguration('is_vulkan')
    enable_api_control = LaunchConfiguration('enable_api_control')
    enable_object_transforms_list = LaunchConfiguration('enable_object_transforms_list')
    output = LaunchConfiguration('output')

    vehicle_name = LaunchConfiguration('vehicle_name')
    max_xy_speed = LaunchConfiguration('max_xy_speed')
    max_z_speed = LaunchConfiguration('max_z_speed')
    max_yaw_rate_deg_s = LaunchConfiguration('max_yaw_rate_deg_s')

    airsim_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('airsim_ros_pkgs'),
            'launch',
            'airsim_node.launch.py'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('host_ip', default_value='localhost'),
        DeclareLaunchArgument('host_port', default_value='41451'),
        DeclareLaunchArgument('publish_clock', default_value='False'),
        DeclareLaunchArgument('is_vulkan', default_value='True'),
        DeclareLaunchArgument('enable_api_control', default_value='False'),
        DeclareLaunchArgument('enable_object_transforms_list', default_value='True'),
        DeclareLaunchArgument('output', default_value='screen'),

        DeclareLaunchArgument('vehicle_name', default_value='Drone1'),
        DeclareLaunchArgument('max_xy_speed', default_value='5.0'),
        DeclareLaunchArgument('max_z_speed', default_value='2.0'),
        DeclareLaunchArgument('max_yaw_rate_deg_s', default_value='90.0'),


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

        # AirSim simple actions node
        Node(
            package='bt_airsim_ros',
            executable='airsim_simple_actions',
            name='airsim_simple_actions',
            output=output,
            parameters=[{
                'host_ip': host_ip,
                'vehicle_name': vehicle_name,
                'max_xy_speed': max_xy_speed,
                'max_z_speed': max_z_speed,
                'max_yaw_rate_deg_s': max_yaw_rate_deg_s,
                'start_up_action_name': [
                    TextSubstitution(text='airsim/'),
                    vehicle_name,
                    TextSubstitution(text='/start_up'),
                ],
                'move_direction_action_name': [
                    TextSubstitution(text='airsim/'),
                    vehicle_name,
                    TextSubstitution(text='/move_direction'),
                ],
                'turn_camera_action_name': [
                    TextSubstitution(text='airsim/'),
                    vehicle_name,
                    TextSubstitution(text='/turn_camera'),
                ],

            }],
        ),
    ])
