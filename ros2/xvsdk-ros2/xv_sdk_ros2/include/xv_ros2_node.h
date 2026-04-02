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
#include "xv_ros2_msgs/srv/save_map_and_switch_cslam.hpp"
#include "xv_ros2_msgs/srv/load_map_and_switch_cslam.hpp"
#include "xv_ros2_msgs/msg/color_depth.hpp"
#include "xv_ros2_msgs/srv/controller_start.hpp"
#include "xv_ros2_msgs/srv/controller_stop.hpp"
#include "xv_ros2_msgs/msg/controller.hpp"
#include "xv_ros2_msgs/msg/event_data.hpp"
#include "xv_ros2_msgs/msg/button_msg.hpp"
#include <image_transport/image_transport.hpp>
#include "xv_dev_wrapper.h"
using namespace xv;
using rosImage = sensor_msgs::msg::Image;
using rosCamInfo = sensor_msgs::msg::CameraInfo;
class xvision_ros2_node : public rclcpp::Node {
public:
    xvision_ros2_node(void);
    void init(void);
    void initTopicAndServer(std::string sn);
    void initControllerTopicAndServer(std::string sn);
    void initFrameIDParameter(void);
    void get_frameId_parameters(void);
    void get_device_config_parameters(void);
    void printInfoMsg(const std::string msgString) const;
    void printWarningMsg(const std::string msgString) const;
    void printErrorMsg(const std::string msgString) const;
    void publishImu(std::string sn, const sensor_msgs::msg::Imu &imuMsg);
    void publisheOrientation(std::string sn, const xv_ros2_msgs::msg::OrientationStamped& orientationMsg);
    void publishFEImage(std::string sn, const rosImage& image, const rosCamInfo& cameraInfo, xv_dev_wrapper::FE_IMAGE_TYPE imageType);
    void publishFEAntiDistortionImage(std::string sn, const rosImage& image, const rosCamInfo& cameraInfo, xv_dev_wrapper::FE_IMAGE_TYPE imageType);
    void publishSlamPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose);
    void publishSlamTrajectory(std::string sn, const nav_msgs::msg::Path& path);
    void publishTofCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishRGBCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishSGBMImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishSGBMRawImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishRGBDCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishRGBDRawCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo);
    void publishRGBDRawCameraData(std::string sn, const xv_ros2_msgs::msg::ColorDepth& orientationMsg);
    std::string getFrameID(const std::string &defaultId);
    bool getConfig(std::string configName);
    void broadcasterTfTransform(const geometry_msgs::msg::TransformStamped & transform);
    void publishLeftControllerPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose);
    void publishRightControllerPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose);
    void publishLeftControllerData(std::string sn, const xv_ros2_msgs::msg::Controller& controllerMsg);
    void publishRightControllerData(std::string sn, const xv_ros2_msgs::msg::Controller& controllerMsg);
    void publishEvent(std::string sn, const xv_ros2_msgs::msg::EventData& eventMsg);
    void publishButton(std::string sn, int buttonType, const xv_ros2_msgs::msg::ButtonMsg& buttonMsg);
    void broadcasterStaticTfTransform(const geometry_msgs::msg::TransformStamped & transform);
    bool isFramePublished(const std::string& frame_id);
    void setFramePublished(const std::string& frame_id);

private:
    void watchDevices(void);
    void watchControllers(void);

private:
    // imu publisher and service.
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> m_imuPublisher;

    // orientationStream publisher and service.
    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::OrientationStamped>::SharedPtr> m_orientationPublisher;
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_imuSensor_startOri;
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_imuSensor_stopOri;
    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::GetOrientation>::SharedPtr> m_service_imuSensor_getOri;
    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::GetOrientationAt>::SharedPtr> m_service_imuSensor_getOriAt;

    // FE publisher and service.
    std::map<std::string, image_transport::Publisher>  m_fisheyeImageLeftPublisher;
    std::map<std::string, image_transport::Publisher>  m_fisheyeImageRightPublisher;
    std::map<std::string, image_transport::Publisher>  m_fisheyeAntiDistortionLeftImagePublisher;
    std::map<std::string, image_transport::Publisher>  m_fisheyeAntiDistortionRightImagePublisher;
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_fisheyeImagePublisher[2];
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>  m_fisheyeImageLeftInfoPublisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>  m_fisheyeImageRightInfoPublisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>  m_fisheyeAntiDistortionCamLeftInfoPublisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>  m_fisheyeAntiDistortionCamRightInfoPublisher;

    //slam publisher and service.
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_slam_pose_publisher;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> m_slam_path_publisher;
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_slam_start;
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_slam_stop;
    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::GetPose>::SharedPtr> m_service_slam_get_pose;
    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::GetPoseAt>::SharedPtr> m_service_slam_get_pose_at;

    //tf2 static transform broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

    //tof publisher and service.
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_tof_start;
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_tof_stop;
    std::map<std::string, image_transport::Publisher> m_tof_camera_publisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> m_tof_cameraInfo_pub;

    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_rgb_start;
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> m_service_rgb_stop;
    std::map<std::string, image_transport::Publisher> m_rgb_camera_publisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> m_rgb_cameraInfo_pub;

    std::map<std::string, image_transport::Publisher> m_sgbm_camera_publisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> m_sgbm_cameraInfo_pub;
    std::map<std::string, image_transport::Publisher> m_sgbm_raw_camera_publisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> m_sgbm_raw_cameraInfo_pub;

    std::map<std::string, image_transport::Publisher> m_rgbd_camera_publisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> m_rgbd_cameraInfo_pub;

    std::map<std::string, image_transport::Publisher> m_rgbd_raw_camera_publisher;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> m_rgbd_raw_cameraInfo_pub;

    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::ColorDepth>::SharedPtr> m_rgbd_raw_data_publisher;

    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::SaveMapAndSwitchCslam>::SharedPtr> m_cslam_save_map;
    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::LoadMapAndSwitchCslam>::SharedPtr> m_cslam_load_map;

    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::ControllerStart>::SharedPtr> m_controller_start;
    std::map<std::string, rclcpp::Service<xv_ros2_msgs::srv::ControllerStop>::SharedPtr> m_controller_stop;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_controller_left_pose_publisher;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_controller_right_pose_publisher;
    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::Controller>::SharedPtr> m_controller_left_data_publisher;
    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::Controller>::SharedPtr> m_controller_right_data_publisher;

    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::EventData>::SharedPtr> m_eventPublisher;

    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr> m_buttonPublisher1;
    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr> m_buttonPublisher2;
    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr> m_buttonPublisher3;
    std::map<std::string, rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr> m_buttonPublisher4;

    size_t count_;
    std::thread m_watchDevThread, m_watchControllerThread;
    bool m_stopWatchDevices;
    std::shared_ptr<xv_dev_wrapper> m_device, m_controllerDevice;
    std::string m_devSerialNumber;
	std::string m_controllerSerialNumber = "Controller";
    std::vector<std::string> m_serialNumbers, m_controllerSerials;
    std::map<std::string, std::shared_ptr<xv_dev_wrapper>> m_deviceMap, m_controllerMap;
    std::map<std::string, std::string> m_frameID;
    std::map<std::string, bool> m_deviceConfig;
    std::shared_ptr<xv::Device> controllerDevice;
    std::string controllerSN;
    std::set<std::string> m_published_frames;

    // Remapped pose publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_remapped_pose_publisher;
};
#endif
