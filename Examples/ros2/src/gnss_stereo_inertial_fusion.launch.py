import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'gnss_stereo_inertial_fusion_ros2'
    share_dir = get_package_share_directory(package_name)

    # Path to the vocabulary file and settings file
    # These should be provided by the user or located in a known directory
    # For now, we'll use placeholders assuming they are in a 'config' directory within the package share directory
    vocabulary_path = os.path.join(share_dir, 'config', 'ORBvoc.txt') # Example, replace with actual voc file
    settings_path = os.path.join(share_dir, 'config', 'stereo_imu_gps.yaml') # Example, replace with actual settings file

    return LaunchDescription([
        Node(
            package=package_name,
            executable='gnss_stereo_inertial_fusion_node',
            name='gnss_stereo_inertial_fusion_node',
            output='screen',
            parameters=[
                {'vocabulary_path': vocabulary_path},
                {'settings_path': settings_path},
            ],
            remappings=[
                ('/camera/left/image_raw', '/stereo_camera/left/image_raw'),
                ('/camera/right/image_raw', '/stereo_camera/right/image_raw'),
                ('/imu/data', '/imu/data'),
                ('/gps/fix', '/gps/fix'),
                ('/orb_slam3/pose', '/orb_slam3/pose'),
            ]
        )
    ])


