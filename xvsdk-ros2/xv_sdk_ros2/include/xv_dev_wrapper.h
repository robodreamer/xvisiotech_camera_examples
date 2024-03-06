#ifndef __XV_DEV_WRAPPER_H__
#define __XV_DEV_WRAPPER_H__

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <xv-sdk.h>
#include <xv-sdk-ex.h>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "nav_msgs/msg/path.hpp"
#include "xv_ros2_msgs/msg/orientation_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class xvision_ros2_node;

using namespace xv;
class xv_dev_wrapper
{
public:
    explicit xv_dev_wrapper(xvision_ros2_node* node, std::shared_ptr<xv::Device> device);
    void init(void);
    bool startImuOri(void);
    bool stopImuOri(void);
    bool getImuOri(xv_ros2_msgs::msg::OrientationStamped& oriStamped, const builtin_interfaces::msg::Duration& duration);
    bool getImuOriAt(xv_ros2_msgs::msg::OrientationStamped& oriStamped, const builtin_interfaces::msg::Time &time);
    bool start_slam(void);
    bool stop_slam(void);
    bool slam_get_pose(geometry_msgs::msg::PoseStamped& poseSteamped, const builtin_interfaces::msg::Duration& prediction);
    bool slam_get_pose_at(geometry_msgs::msg::PoseStamped& poseSteamped, const builtin_interfaces::msg::Time& time);
    bool start_tof(void);
    bool stop_tof(void);
    bool start_rgb(void);
    bool stop_rgb(void);

public:
    enum FE_IMAGE_TYPE
    {
        LEFT_IMAGE = 0,
        RIGHT_IMAGE = 1
    };

private:
    void initImu(void);
    void initOrientationStream(void);
    void initFisheyeCameras(void);
    void initSlam(void);
    void initTofCamera();
    void initColorCamera();
    void formatImuTopicMsg(sensor_msgs::msg::Imu &rosImu, const xv::Imu &xvImu);
    void formatXvOriToRosOriStamped(xv_ros2_msgs::msg::OrientationStamped &rosOrientation,
                                    const xv::Orientation& xvOrientation,
                                    const std::string& frameId);
    rclcpp::Time getHeaderStamp(double hostTimesStamp);
    sensor_msgs::msg::CameraInfo toRosCameraInfo(const xv::UnifiedCameraModel* const ucm,
                                                 const xv::PolynomialDistortionCameraModel* const pdcm);
    sensor_msgs::msg::CameraInfo toRosCameraInfo(const xv::SpecialUnifiedCameraModel* const seucm);
    bool registerFECallbackFunc(void);
    bool registerFEAntiDistortionCallbackFunc(void);
    bool registerSGBMCallbackFunc(void);
    sensor_msgs::msg::Image changeFEGrayScaleImage2RosImage(const GrayScaleImage& xvGrayImage, double timestamp, const std::string& frame_id);
    void getFECalibration();
    double get_sec(const builtin_interfaces::msg::Duration& prediction)const;
    double get_sec(const builtin_interfaces::msg::Time& timestamp)const;
    geometry_msgs::msg::PoseStamped to_ros_poseStamped(const Pose& xvPose, const std::string& frame_id);
    builtin_interfaces::msg::Time get_stamp_from_sec(double sec) const;
    double steady_clock_now();
    nav_msgs::msg::Path toRosPoseStampedRetNavmsgs(const Pose& xvPose, const std::string& frame_id, nav_msgs::msg::Path& path);
    geometry_msgs::msg::TransformStamped toRosTransformStamped(const Pose& pose,
                                                      const std::string& parent_frame_id,
                                                      const std::string& frame_id);
    sensor_msgs::msg::Image toRosImage(const DepthImage& xvDepthImage, const std::string& frame_id);
    sensor_msgs::msg::Image toRosImage(const ColorImage& xvColorImage, const std::string& frame_id);
    sensor_msgs::msg::Image toRosImage(const SgbmImage& xvSgbmDepthImage, const std::string& frame_id);
    cv::Mat toCvMatRGB(const ColorImage& xvColorImage);
    void toRosOrientationStamped(xv_ros2_msgs::msg::OrientationStamped& orientation, Orientation const& xvOrientation, const std::string& frame_id);

private:
    xvision_ros2_node* m_node;
    std::shared_ptr<xv::Device> m_device;
    std::vector<xv::Calibration> m_xvFisheyesCalibs;
    std::vector<std::map<int /*height*/, sensor_msgs::msg::CameraInfo>> m_fisheyeCameraInfos;
    bool m_slam_pose_enable;
    bool m_slam_path_enable;
    bool m_fisheye_enable;
    sensor_msgs::msg::CameraInfo m_tofCameraInfo;
    sensor_msgs::msg::CameraInfo m_rgbCameraInfo;
    xv::Calibration m_xvTofCalib;
    xv::Calibration m_xvRGBCalib;
    xv::CalibrationEx m_xvSgbmCalib;
    sensor_msgs::msg::CameraInfo m_sgbmCamInfo;
};
#endif
