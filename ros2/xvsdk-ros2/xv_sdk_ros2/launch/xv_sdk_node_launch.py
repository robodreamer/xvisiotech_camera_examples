from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch_ros.actions

def generate_launch_description():
    slam_path_enable = launch.substitutions.LaunchConfiguration('slam_path_enable', default='false')
    slam_pose_enable = launch.substitutions.LaunchConfiguration('slam_pose_enable', default='true')
    fisheye_enable = launch.substitutions.LaunchConfiguration('fisheye_enable', default='false')
    button_states_enable = launch.substitutions.LaunchConfiguration('button_states_enable', default='true')
    rgb_enable = launch.substitutions.LaunchConfiguration('rgb_enable', default='false')
    tof_enable = launch.substitutions.LaunchConfiguration('tof_enable', default='false')
    rgbd_enable = launch.substitutions.LaunchConfiguration('rgbd_enable', default='false')
    event_enable = launch.substitutions.LaunchConfiguration('event_enable', default='false')
    orientation_enable = launch.substitutions.LaunchConfiguration('orientation_enable', default='false')

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
                {"fisheye_enable": fisheye_enable},
                {"button_states_enable": button_states_enable},
                {"rgb_enable": rgb_enable},
                {"tof_enable": tof_enable},
                {"rgbd_enable": rgbd_enable},
                {"event_enable": event_enable},
                {"orientation_enable": orientation_enable},
                {"controller_port": "/dev/ttyUSB0"}
            ]
        )
    ])
