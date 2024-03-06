#ifndef __XV_ROS2_NODE_H__
#define __XV_ROS2_NODE_H__
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <xv-sdk.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "xv_ros2_msgs/msg/orientation_stamped.hpp"
#include "xv_ros2_msgs/srv/get_orientation.hpp"
#include "xv_ros2_msgs/srv/get_orientation_at.hpp"
#include "xv_ros2_msgs/srv/get_pose.hpp"
#include "xv_ros2_msgs/srv/get_pose_at.hpp"
#include <image_transport/image_transport.h>
#include "xv_dev_wrapper.h"
using namespace xv;
using rosImage = sensor_msgs::msg::Image;
using rosCamInfo = sensor_msgs::msg::CameraInfo;
class xvision_ros2_node : public rclcpp::Node {
public:
    xvision_ros2_node(void);
    void init(void);
    void initTopicAndServer(void);
    void initFrameIDParameter(void);
    void get_frameId_parameters(void);
    void get_device_config_parameters(void);
    void printInfoMsg(const std::string msgString) const;
    void printErrorMsg(const std::string msgString) const;
    void publishImu(const sensor_msgs::msg::Imu &imuMsg);
    void publisheOrientation(const xv_ros2_msgs::msg::OrientationStamped& orientationMsg);
    void publishFEImage(const rosImage& image, const rosCamInfo& cameraInfo, xv_dev_wrapper::FE_IMAGE_TYPE imageType);
    void publishFEAntiDistortionImage(const rosImage& image, const rosCamInfo& cameraInfo, xv_dev_wrapper::FE_IMAGE_TYPE imageType);
    void publishSlamPose(const geometry_msgs::msg::PoseStamped& pose);
    void publishSlamTrajectory(const nav_msgs::msg::Path& path);
    void publishTofCameraImage(const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishRGBCameraImage(const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishSGBMImage(const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    std::string getFrameID(const std::string &defaultId);
    bool getConfig(std::string configName);
    void broadcasterTfTransform(const geometry_msgs::msg::TransformStamped & transform);

private:
    void watchDevices(void);

private:
    // imu publisher and service.
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
    
    // orientationStream publisher and service.
    rclcpp::Publisher<xv_ros2_msgs::msg::OrientationStamped>::SharedPtr m_orientationPublisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_imuSensor_startOri;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_imuSensor_stopOri;
    rclcpp::Service<xv_ros2_msgs::srv::GetOrientation>::SharedPtr m_service_imuSensor_getOri;
    rclcpp::Service<xv_ros2_msgs::srv::GetOrientationAt>::SharedPtr m_service_imuSensor_getOriAt;
    
    // FE publisher and service.
    image_transport::Publisher  m_fisheyeImagePublisher[2];
    image_transport::Publisher  m_fisheyeAntiDistortionImagePublisher[2];
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_fisheyeImagePublisher[2];
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_fisheyeCamInfoPublisher[2];
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_fisheyeAntiDistortionCamInfoPublisher[2];

    //slam publisher and service.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_slam_pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_slam_path_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_slam_start;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_slam_stop;
    rclcpp::Service<xv_ros2_msgs::srv::GetPose>::SharedPtr m_service_slam_get_pose;
    rclcpp::Service<xv_ros2_msgs::srv::GetPoseAt>::SharedPtr m_service_slam_get_pose_at;

    //tf2 static transform broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;
    
    //tof publisher and service.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_tof_start;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_tof_stop;
    image_transport::Publisher m_tof_camera_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_tof_cameraInfo_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_rgb_start;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_rgb_stop;
    image_transport::Publisher m_rgb_camera_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_rgb_cameraInfo_pub;

    image_transport::Publisher m_sgbm_camera_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_sgbm_cameraInfo_pub;

    size_t count_;
    std::thread m_watchDevThread;
    bool m_stopWatchDevices;
    std::shared_ptr<xv_dev_wrapper> m_device;
    std::string m_devSerialNumber;
    std::map<std::string, std::shared_ptr<xv_dev_wrapper>> m_deviceMap;
    std::map<std::string, std::string> m_frameID;
    std::map<std::string, bool> m_deviceConfig;
};
#endif