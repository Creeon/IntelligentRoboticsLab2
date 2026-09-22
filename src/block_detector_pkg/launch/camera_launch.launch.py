from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Left camera
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='left_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'frame_rate': 30.0,
                'camera_frame_id': 'left_camera',
                'camera_info_url':
                    'file:///home/anc5gg/dev/lab2_ws/calibration_data/mono_left_calibration_data2/ost.yaml'
            }],
            remappings=[
                ('image_raw', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
            ]
        ),

        # Right camera
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='right_camera',
            parameters=[{
                'video_device': '/dev/video2',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'frame_rate': 30.0,
                'camera_frame_id': 'right_camera',
                'camera_info_url':
                    'file:///home/anc5gg/dev/lab2_ws/calibration_data/mono_right_calibration_data2/ost.yaml'
            }],
            remappings=[
                ('image_raw', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
            ]
        ),

        # Left rectification
        Node(
            package='image_proc',
            executable='image_proc',
            name='left_image_proc',
            remappings=[
                ('image', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image_rect', '/stereo/left/image_rect'),
            ]
        ),

        # Right rectification
        Node(
            package='image_proc',
            executable='image_proc',
            name='right_image_proc',
            remappings=[
                ('image', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
                ('image_rect', '/stereo/right/image_rect'),
            ]
        ),

        # AprilTag detection using left camera
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_left',
            parameters=[
                '/home/anc5gg/dev/lab2_ws/calibration_data/apriltag_calibration/apriltag_params.yaml'
            ],
            remappings=[
                ('image_rect', '/stereo/left/image_rect'),
                ('camera_info', '/stereo/left/camera_info'),
            ]
        ),
        
        # AprilTag detection using right camera
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_right',
            parameters=[
                '/home/anc5gg/dev/lab2_ws/calibration_data/apriltag_calibration/apriltag_params.yaml'
            ],
            remappings=[
                ('image_rect', '/stereo/right/image_rect'),
                ('camera_info', '/stereo/right/camera_info'),
            ]
        ),
    ])