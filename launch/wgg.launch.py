from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_web_package',
            executable='web_node',
            name='web_node',
            output='screen',
            # parameters=[{
            #     'param_name': 'param_value'
            # }]
        ),
        Node(
            package='my_web_package',
            executable='service_bridge',
            name='service_bridge',
            output='screen',
            # parameters=[{
            #     'param_name': 'param_value'
            # }]
        ),
    ])
