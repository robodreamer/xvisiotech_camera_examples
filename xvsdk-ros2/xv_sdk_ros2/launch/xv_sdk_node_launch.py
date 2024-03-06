from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch_ros.actions

def generate_launch_description():
    #imu_linear_acceleration = launch.substitutions.LaunchConfiguration('imu_linear_acceleration', default='0.0')
    #imu_angular_velocity = launch.substitutions.LaunchConfiguration('imu_angular_velocity', default='0.0')
    slam_path_enable = launch.substitutions.LaunchConfiguration('slam_path_enable', default='true')
    slam_pose_enable = launch.substitutions.LaunchConfiguration('slam_pose_enable', default='true')                                                              
    fisheye_enable = launch.substitutions.LaunchConfiguration('fisheye_enable', default='false')                                                              

    return LaunchDescription([
        Node(
            package="xv_sdk_ros2",
            executable="xv_cameras",
            name="xv_cameras",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"map_optical_frame": "xv_sdk/map_optical_frame"},
                {"imu_optical_frame": "xv_sdk/imu_optical_frame"},
                {"imu_optical_frame_unfiltered": "imu_optical_frame_unfiltered"},
                {"fisheye_left_optical_frame": "fisheye_left_optical_frame"},
                {"fisheye_right_optical_frame": "fisheye_right_optical_frame"},
                {"color_optical_frame": "color_optical_frame"},
                {"sgbm_optical_frame": "sgbm_optical_frame"},
                {"tof_optical_frame": "tof_optical_frame"},
                {"imu_link": "imu_link"},
                {"base_link": "base_link"},
                {"odom": "odom"},
                {"imu_linear_acceleration": 0.0},
                {"imu_angular_velocity": 0.0},
                {"slam_path_enable": slam_path_enable},
                {"slam_pose_enable": slam_pose_enable},
                {"fisheye_enable": fisheye_enable}
            ],
            remappings=[
                #topic
                ('imu', '/xv_sdk/imu'),
                ('orientation', '/xv_sdk/orientation'),
                ('pose', 'xv_sdk/slam/pose'),
                ('trajectory', 'xv_sdk/slam/trajectory'),
                ('fisheye_cameras_left/image', 'xv_sdk/fisheye_cameras_left/image'),
                ('fisheye_cameras_right/image', 'xv_sdk/fisheye_cameras_right/image'),
                ('fisheye_cameras_left/camera_info', 'xv_sdk/fisheye_cameras_left/camera_info'),
                ('fisheye_cameras_right/camera_info', 'xv_sdk/fisheye_cameras_right/camera_info'),
                ('fisheye_AntiDistortion_cameras_left/image', 'xv_sdk/fisheye_AntiDistortion_cameras_left/image'),
                ('fisheye_AntiDistortion_cameras_right/image', 'xv_sdk/fisheye_AntiDistortion_cameras_right/image'),
                ('fisheye_AntiDistortion_cameras_left/camera_info', 'xv_sdk/fisheye_AntiDistortion_cameras_left/camera_info'),
                ('fisheye_AntiDistortion_cameras_right/camera_info', 'xv_sdk/fisheye_AntiDistortion_cameras_right/camera_info'),
                ('tof/depth/image_rect_raw','xv_sdk/tof/depth/image_rect_raw'),
                ('tof/depth/camera_info', 'xv_sdk/tof/depth/camera_info'),
                ('rgb/image', 'xv_sdk/rgb/image'),
                ('rgb/camera_info', 'xv_sdk/rgb/camera_info'),
                # server
                ('start_orientation', 'xv_sdk/service/orientation/start_orientation'),
                ('stop_orientation', 'xv_sdk/service/orientation/stop_orientation'),
                ('get_orientation', 'xv_sdk/service/orientation/get_orientation'),
                ('get_orientation_at', 'xv_sdk/service/orientation/get_orientation_at'),
                ('start_slam', 'xv_sdk/service/slam/start_slam'),
                ('stop_slam', 'xv_sdk/service/slam/stop_slam'),
                ('get_pose', 'xv_sdk/service/slam/get_pose'),
                ('start_tof','xv_sdk/service/tof/start_tof'),
                ('stop_tof','xv_sdk/service/tof/stop_tof'),
                ('start_rgb','xv_sdk/service/rgb/start_rgb'),
                ('stop_rgb','xv_sdk/service/rgb/stop_rgb')
            ]
        )
    ])
