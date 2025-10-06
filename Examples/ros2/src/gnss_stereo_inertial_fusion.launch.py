from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_stereo_inertial_fusion_ros2',
            executable='gnss_stereo_inertial_fusion_node',
            name='gnss_stereo_inertial_fusion_node',
            output='screen',
            parameters=[
                # TODO: Add parameters from the original ROS1 launch file
                # Example: {'param_name': 'param_value'}
            ],
            remappings=[
                # TODO: Add remappings for topics if necessary
                # Example: ('/old_topic', '/new_topic')
            ]
        )
    ])
