from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main',
            executable='camera',
            name='camera_node',
            arguments=['-v', '0']
        ),
        Node(
            package='main',
            executable='command_handler',
            name='command_handler_node'
        ),
        # Node(
        #     package='main',
        #     executable='yolo_detection',
        #     name='detection_node'
        # ),
        Node(
            package='main',
            executable='imageproc',
            name='image_processing_node'
        ),
        Node(
            package='main',
            executable='serial',
            name='serial_node'
        ),
        Node(
            package='main',
            executable='tracking',
            name='tracking_node'
        ),
        Node(
            package='main',
            executable='health_monitoring',
            name='health_monitoring_node'
        ),
        Node(
            package='main',
            executable='ui',
            name='ui_node'
        )
    ])
