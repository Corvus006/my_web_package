from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_web_package',
            executable='web_node',
            name='web_node',
            output='screen',
            parameters=[
                {'front_topic': '/wgg/front/compressed'},
                {'rear_topic': '/wgg/rear/compressed'},
                {'gripper_topic': '/wgg/gripper/compressed'},
                {'thermal_topic': '/wgg/thermal/compressed'},
                {'motion_topic': '/wgg/motion/compressed'},
                {'cmd_vel_topic': '/wgg/cmd_vel'},
                {'enable_service': '/wgg/enable'},
                # Add other parameters specific to `web_node` here
            ]
        ),
        Node(
            package='my_web_package',
            executable='service_bridge',
            name='service_bridge',
            output='screen',
            parameters=[
                {'enable_service_name': '/wgg/enable'},
                {'set_mode_service_name': '/wgg/set_mode'},
                # Add other parameters specific to `service_bridge` here
            ]
        ),
    ])
