#include "xv_ros2_node.h"
xvision_ros2_node::xvision_ros2_node(void) :
    Node("xvisio_SDK", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)),
    count_(0),
    m_stopWatchDevices(false)
{

}

void xvision_ros2_node::init(void)
{
    //initFrameIDParameter();
    get_frameId_parameters();
    get_device_config_parameters();
    // initTopicAndServer();

    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    m_watchDevThread = std::thread(&xvision_ros2_node::watchDevices, this);
    m_watchControllerThread = std::thread(&xvision_ros2_node::watchControllers, this);
}

void xvision_ros2_node::initTopicAndServer(std::string sn)
{
    std::string imuPubName = "xv_sdk/SN" + sn +"/imu";
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    m_imuPublisher.emplace(sn, imuPublisher);
    m_imuPublisher[sn] = this->create_publisher<sensor_msgs::msg::Imu>(imuPubName.c_str(), 10);
    std::string oriPubName = "xv_sdk/SN" + sn +"/orientation";
    rclcpp::Publisher<xv_ros2_msgs::msg::OrientationStamped>::SharedPtr orienPublisher;
    m_orientationPublisher.emplace(sn, orienPublisher);
    m_orientationPublisher[sn] = this->create_publisher<xv_ros2_msgs::msg::OrientationStamped>(oriPubName.c_str(),1);
    auto imuSensor_startOri_callBack =[this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool success = m_deviceMap[sn] ? m_deviceMap[sn]->startImuOri() : false;
        res->success = success;
        res->message = success ? "successed" : "failed";
        return success;
    };
    std::string oriStart = "xv_sdk/SN" + sn +"/start_orientation";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startOrienSrv;
    m_service_imuSensor_startOri.emplace(sn, startOrienSrv);
    m_service_imuSensor_startOri[sn] = this->create_service<std_srvs::srv::Trigger>(oriStart.c_str(), imuSensor_startOri_callBack);

    auto imuSensor_stopOri_callBack =[this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool success =  m_deviceMap[sn] ? m_deviceMap[sn]->stopImuOri() : false;
        res->success = success;
        res->message = success ? "successed" : "failed";
        return success;
    };
    std::string oriStop = "xv_sdk/SN" + sn +"/stop_orientation";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopOrienSrv;
    m_service_imuSensor_stopOri.emplace(sn, stopOrienSrv);
    m_service_imuSensor_stopOri[sn] = this->create_service<std_srvs::srv::Trigger>(oriStop.c_str(), imuSensor_stopOri_callBack);

    auto imuSensor_getOri_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::GetOrientation_Request> req,
                                          std::shared_ptr<xv_ros2_msgs::srv::GetOrientation_Response> res) -> bool
    {
        return m_deviceMap[sn] ? m_deviceMap[sn]->getImuOri(res->orientation, req->prediction) : false;
    };
    std::string getOri = "xv_sdk/SN" + sn +"/get_orientation";
    rclcpp::Service<xv_ros2_msgs::srv::GetOrientation>::SharedPtr oriGetCallback;
    m_service_imuSensor_getOri.emplace(sn, oriGetCallback);
    m_service_imuSensor_getOri[sn] = this->create_service<xv_ros2_msgs::srv::GetOrientation>(getOri.c_str(), imuSensor_getOri_callback);

    auto imuSensor_getOriAt_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::GetOrientationAt_Request> req,
                                    std::shared_ptr<xv_ros2_msgs::srv::GetOrientationAt_Response> res) -> bool
    {
        return m_deviceMap[sn] ? m_deviceMap[sn]->getImuOriAt(res->orientation, req->timestamp) : false;
    };
    std::string getOriAt = "xv_sdk/SN" + sn +"/get_orientation_at";
    rclcpp::Service<xv_ros2_msgs::srv::GetOrientationAt>::SharedPtr oriGetAtCallback;
    m_service_imuSensor_getOriAt.emplace(sn, oriGetAtCallback);
    m_service_imuSensor_getOriAt[sn] = this->create_service<xv_ros2_msgs::srv::GetOrientationAt>(getOriAt.c_str(), imuSensor_getOriAt_callback);

    std::string feImage0 = "xv_sdk/SN" + sn +"/fisheye_cameras_left/image";
    image_transport::Publisher feImageLeftPub;
    m_fisheyeImageLeftPublisher.emplace(sn, feImageLeftPub);
    m_fisheyeImageLeftPublisher[sn] = image_transport::create_publisher(
        this, feImage0.c_str());//this->create_publisher<sensor_msgs::msg::Image>("", 10);
    std::string feInfo0 = "xv_sdk/SN" + sn +"/fisheye_cameras_left/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feImageLeftInfoPub;
    m_fisheyeImageLeftInfoPublisher.emplace(sn, feImageLeftInfoPub);
    m_fisheyeImageLeftInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfo0.c_str(), 1);

    std::string feImage1 = "xv_sdk/SN" + sn +"/fisheye_cameras_right/image";
    image_transport::Publisher feImageRightPub;
    m_fisheyeImageRightPublisher.emplace(sn, feImageRightPub);
    m_fisheyeImageRightPublisher[sn] = image_transport::create_publisher(
        this, feImage1.c_str());//this->create_publisher<sensor_msgs::msg::Image>("fisheye_cameras_right/image", 10);
    std::string feInfo1 = "xv_sdk/SN" + sn +"/fisheye_cameras_right/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feImagRightInfoPub;
    m_fisheyeImageRightInfoPublisher.emplace(sn, feImagRightInfoPub);
    m_fisheyeImageRightInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfo1.c_str(), 1);

    std::string feImageAnti0 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_left/image";
    image_transport::Publisher feAntiImageLeftPub;
    m_fisheyeAntiDistortionLeftImagePublisher.emplace(sn, feAntiImageLeftPub);
    m_fisheyeAntiDistortionLeftImagePublisher[sn] = image_transport::create_publisher(
        this, feImageAnti0.c_str());//this->create_publisher<sensor_msgs::msg::Image>("", 10);
    std::string feInfoAnti0 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_left/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feAntiImageLeftInfoPub;
    m_fisheyeAntiDistortionCamLeftInfoPublisher.emplace(sn, feAntiImageLeftInfoPub);
    m_fisheyeAntiDistortionCamLeftInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfoAnti0.c_str(), 1);

    std::string feImageAnti1 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_right/image";
    image_transport::Publisher feAntiImageRightPub;
    m_fisheyeAntiDistortionRightImagePublisher.emplace(sn, feAntiImageRightPub);
    m_fisheyeAntiDistortionRightImagePublisher[sn] = image_transport::create_publisher(
        this, feImageAnti1.c_str());//this->create_publisher<sensor_msgs::msg::Image>("fisheye_cameras_right/image", 10);
    std::string feInfoAnti1 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_right/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feAntiImageRightInfoPub;
    m_fisheyeAntiDistortionCamRightInfoPublisher.emplace(sn, feAntiImageRightInfoPub);
    m_fisheyeAntiDistortionCamRightInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfoAnti1.c_str(), 1);

    auto slam_start_service_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool success = m_deviceMap[sn] ? m_deviceMap[sn]->start_slam() : false;
        res->success = success;
        res->message = success ? "successed" : "failed";
        return success;
    };
    std::string startSlam = "xv_sdk/SN" + sn +"/start_slam";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr slamStart;
    m_service_slam_start.emplace(sn, slamStart);
    m_service_slam_start[sn] = this->create_service<std_srvs::srv::Trigger>(startSlam.c_str(), slam_start_service_callback);

    auto slam_stop_service_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool success = m_deviceMap[sn] ?m_deviceMap[sn]->stop_slam() : false;
        res->success = success;
        res->message = success ? "successed" : "failed";
        return success;
    };
    std::string stopSlam = "xv_sdk/SN" + sn +"/stop_slam";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr slamStop;
    m_service_slam_stop.emplace(sn, slamStop);
    m_service_slam_stop[sn] = this->create_service<std_srvs::srv::Trigger>(stopSlam.c_str(), slam_stop_service_callback);

    auto slam_get_pose_service_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::GetPose::Request> req,
                                        std::shared_ptr<xv_ros2_msgs::srv::GetPose::Response> res) -> bool
    {
        bool success = m_deviceMap[sn] ? m_deviceMap[sn]->slam_get_pose(res->pose, req->prediction) : false;
        return success;
    };

    std::string getPose = "xv_sdk/SN" + sn +"/get_pose";
    rclcpp::Service<xv_ros2_msgs::srv::GetPose>::SharedPtr slamGetPose;
    m_service_slam_get_pose.emplace(sn, slamGetPose);
    m_service_slam_get_pose[sn] = this->create_service<xv_ros2_msgs::srv::GetPose>(getPose.c_str(), slam_get_pose_service_callback);

    auto slam_get_pose_at_service_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::GetPoseAt::Request> req,
                                        std::shared_ptr<xv_ros2_msgs::srv::GetPoseAt::Response> res) -> bool
    {
        bool success = m_deviceMap[sn] ? m_deviceMap[sn]->slam_get_pose_at(res->pose, req->timestamp) : false;
        return success;
    };
    std::string getPoseAt = "xv_sdk/SN" + sn +"/get_pose_at";
    rclcpp::Service<xv_ros2_msgs::srv::GetPoseAt>::SharedPtr slamGetPoseAt;
    m_service_slam_get_pose_at.emplace(sn, slamGetPoseAt);
    m_service_slam_get_pose_at[sn] = this->create_service<xv_ros2_msgs::srv::GetPoseAt>(getPoseAt.c_str(), slam_get_pose_at_service_callback);

    if(m_deviceConfig["slam_pose_enable"])
    {
        std::string slamPose = "xv_sdk/SN" + sn +"/pose";
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr slamPub;
        m_slam_pose_publisher.emplace(sn, slamPub);
        m_slam_pose_publisher[sn] = this->create_publisher<geometry_msgs::msg::PoseStamped>(slamPose.c_str(), 10);
    }

    if(m_deviceConfig["slam_path_enable"])
    {
        std::string trajectory = "xv_sdk/SN" + sn +"/trajectory";
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slamPathPub;
        m_slam_path_publisher.emplace(sn, slamPathPub);
        m_slam_path_publisher[sn] = this->create_publisher<nav_msgs::msg::Path>(trajectory.c_str(), 1);
    }

    // auto tof_start_service_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    //                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    // {
    //     bool success = m_deviceMap[sn] ? m_deviceMap[sn]->start_tof() : false;
    //     res->success = success;
    //     res->message = success ? "successed" : "failed";
    //     return success;
    // };
    // std::string startTOF = "xv_sdk/SN" + sn +"/start_tof";
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tofStart;
    // m_service_tof_start.emplace(sn, tofStart);
    // m_service_tof_start[sn] = this->create_service<std_srvs::srv::Trigger>(startTOF.c_str(), tof_start_service_callback);

    // auto tof_stop_service_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    //                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    // {
    //     bool success = m_deviceMap[sn] ? m_deviceMap[sn]->stop_tof() : false;
    //     res->success = success;
    //     res->message = success ? "successed" : "failed";
    //     return success;
    // };
    // std::string stopTOF = "xv_sdk/SN" + sn +"/stop_tof";
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tofStop;
    // m_service_tof_stop.emplace(sn, tofStop);
    // m_service_tof_stop[sn] = this->create_service<std_srvs::srv::Trigger>(stopTOF.c_str(), tof_stop_service_callback);

    std::string tofImage = "xv_sdk/SN" + sn +"/tof/depth/image_rect_raw";
    image_transport::Publisher tofImagePub;
    m_tof_camera_publisher.emplace(sn, tofImagePub);
    m_tof_camera_publisher[sn] = image_transport::create_publisher(
        this, tofImage.c_str());//image_transport::create_camera_publisher(this, "camera/depth/image_rect_raw");
    std::string tofInfo = "xv_sdk/SN" + sn +"/tof/depth/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr tofImageInfo;
    m_tof_cameraInfo_pub.emplace(sn, tofImageInfo);
    m_tof_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        tofInfo.c_str(), 1);

    // auto rgb_start_service_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    //                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    // {
    //     bool success = m_deviceMap[sn] ? m_deviceMap[sn]->start_rgb() : false;
    //     res->success = success;
    //     res->message = success ? "successed" : "failed";
    //     return success;
    // };
    // std::string startRGB = "xv_sdk/SN" + sn +"/start_rgb";
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rgbStartSrv;
    // m_service_rgb_start.emplace(sn, rgbStartSrv);
    // m_service_rgb_start[sn] = this->create_service<std_srvs::srv::Trigger>(startRGB.c_str(), rgb_start_service_callback);

    // auto rgb_stop_service_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    //                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    // {
    //     bool success = m_deviceMap[sn] ? m_deviceMap[sn]->stop_rgb() : false;
    //     res->success = success;
    //     res->message = success ? "successed" : "failed";
    //     return success;
    // };
    // std::string stopRGB = "xv_sdk/SN" + sn +"/stop_rgb";
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rgbStopSrv;
    // m_service_rgb_stop.emplace(sn, rgbStopSrv);
    // m_service_rgb_stop[sn] = this->create_service<std_srvs::srv::Trigger>(stopRGB.c_str(), rgb_stop_service_callback);

    std::string rgbImage = "xv_sdk/SN" + sn +"/rgb/image";
    image_transport::Publisher rgbImagePub;
    m_rgb_camera_publisher.emplace(sn, rgbImagePub);
    m_rgb_camera_publisher[sn] = image_transport::create_publisher(
        this, rgbImage.c_str());
    std::string rgbInfo = "xv_sdk/SN" + sn +"/rgb/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbImageInfoPub;
    m_rgb_cameraInfo_pub.emplace(sn, rgbImageInfoPub);
    m_rgb_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(rgbInfo.c_str(), 1);

    // std::string sgbmImage = "xv_sdk/SN" + sn +"/sgbm/image";
    // image_transport::Publisher sgbmImagePub;
    // m_sgbm_camera_publisher.emplace(sn, sgbmImagePub);
    // m_sgbm_camera_publisher[sn] = image_transport::create_publisher(
    //     this, sgbmImage.c_str());
    // std::string sgbmInfo = "xv_sdk/SN" + sn +"/sgbm/camera_info";
    // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr sgbmImageInfoPub;
    // m_sgbm_cameraInfo_pub.emplace(sn, sgbmImageInfoPub);
    // m_sgbm_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(sgbmInfo.c_str(), 1);
    // std::string sgbmRawImage = "xv_sdk/SN" + sn +"/sgbm/sgbmRaw/image";
    // image_transport::Publisher sgbmRawImagePub;
    // m_sgbm_raw_camera_publisher.emplace(sn, sgbmRawImagePub);
    // m_sgbm_raw_camera_publisher[sn] = image_transport::create_publisher(
    //     this, sgbmRawImage.c_str());
    // std::string sgbmRawImageInfo = "xv_sdk/SN" + sn +"/sgbm/sgbmRaw/camera_info";
    // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr sgbmRawImageInfoPub;
    // m_sgbm_raw_cameraInfo_pub.emplace(sn, sgbmRawImageInfoPub);
    // m_sgbm_raw_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(sgbmRawImageInfo.c_str(), 1);

    std::string rgbdImage = "xv_sdk/SN" + sn +"/rgbd/image";
    image_transport::Publisher rgbdImagePub;
    m_rgbd_camera_publisher.emplace(sn, rgbdImagePub);
    m_rgbd_camera_publisher[sn] = image_transport::create_publisher(
        this, rgbdImage.c_str());
    std::string rgbdInfo = "xv_sdk/SN" + sn +"/rgbd/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbdImageInfoPub;
    m_rgbd_cameraInfo_pub.emplace(sn, rgbdImageInfoPub);
    m_rgbd_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(rgbdInfo.c_str(), 1);

    std::string rgbdRawImage = "xv_sdk/SN" + sn +"/rgbd/raw/image";
    image_transport::Publisher rgbdRawImagePub;
    m_rgbd_raw_camera_publisher.emplace(sn, rgbdRawImagePub);
    m_rgbd_raw_camera_publisher[sn] = image_transport::create_publisher(
        this, rgbdRawImage.c_str());
    std::string rgbdRawInfo = "xv_sdk/SN" + sn +"/rgbd/raw/camera_info";
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbdRawImageInfoPub;
    m_rgbd_raw_cameraInfo_pub.emplace(sn, rgbdRawImageInfoPub);
    m_rgbd_raw_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(rgbdRawInfo.c_str(), 1);

    std::string rgbdRaw = "xv_sdk/SN" + sn +"/rgbd/raw/data";
    rclcpp::Publisher<xv_ros2_msgs::msg::ColorDepth>::SharedPtr rgbdRawPub;
    m_rgbd_raw_data_publisher.emplace(sn, rgbdRawPub);
    m_rgbd_raw_data_publisher[sn] = this->create_publisher<xv_ros2_msgs::msg::ColorDepth>(rgbdRaw.c_str(),1);

    auto cslam_savemap_service_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::SaveMapAndSwitchCslam::Request> req,
                                        std::shared_ptr<xv_ros2_msgs::srv::SaveMapAndSwitchCslam::Response> res) -> bool
    {
        bool success = m_deviceMap[sn]->saveCslamMap(req->filename);
        if(success)
        {
            res->success = true;
            res->message = "save map file ok";
        }
        else
        {
            res->success = false;
            res->message = "could not save map file";
        }
        return success;
    };
    std::string cslamSave = "xv_sdk/SN" + sn +"/cslam_save_map";
    rclcpp::Service<xv_ros2_msgs::srv::SaveMapAndSwitchCslam>::SharedPtr saveMapSrv;
    m_cslam_save_map.emplace(sn, saveMapSrv);
    m_cslam_save_map[sn] = this->create_service<xv_ros2_msgs::srv::SaveMapAndSwitchCslam>(cslamSave.c_str(), cslam_savemap_service_callback);

    auto cslam_loadmap_service_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::LoadMapAndSwitchCslam::Request> req,
                                        std::shared_ptr<xv_ros2_msgs::srv::LoadMapAndSwitchCslam::Response> res) -> bool
    {
        bool success = m_deviceMap[sn]->loadCslamMap(req->filename);
        if(success)
        {
            res->success = true;
            res->message = "load map file ok";
        }
        else
        {
            res->success = false;
            res->message = "could not load map file";
        }
        return success;
    };
    std::string cslamLoad = "xv_sdk/SN" + sn +"/cslam_load_map";
    rclcpp::Service<xv_ros2_msgs::srv::LoadMapAndSwitchCslam>::SharedPtr loadMapSrv;
    m_cslam_load_map.emplace(sn, loadMapSrv);
    m_cslam_load_map[sn] = this->create_service<xv_ros2_msgs::srv::LoadMapAndSwitchCslam>(cslamLoad.c_str(), cslam_loadmap_service_callback);

    std::string eventName = "xv_sdk/SN" + sn +"/event";
    rclcpp::Publisher<xv_ros2_msgs::msg::EventData>::SharedPtr eventPublisher;
    m_eventPublisher.emplace(sn, eventPublisher);
    m_eventPublisher[sn] = this->create_publisher<xv_ros2_msgs::msg::EventData>(eventName.c_str(),1);

    std::string buttonName1 = "xv_sdk/SN" + sn +"/button1";
    rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr buttonPublisher1;
    m_buttonPublisher1.emplace(sn, buttonPublisher1);
    m_buttonPublisher1[sn] = this->create_publisher<xv_ros2_msgs::msg::ButtonMsg>(buttonName1.c_str(),1);

    std::string buttonName2 = "xv_sdk/SN" + sn +"/button2";
    rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr buttonPublisher2;
    m_buttonPublisher2.emplace(sn, buttonPublisher2);
    m_buttonPublisher2[sn] = this->create_publisher<xv_ros2_msgs::msg::ButtonMsg>(buttonName2.c_str(),1);
}

void xvision_ros2_node::initControllerTopicAndServer(std::string sn)
{
    std::string controllerLeftPose = "xv_sdk/SN" + sn +"/leftControllerPose";
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr leftControllerPosePub;
    m_controller_left_pose_publisher.emplace(sn, leftControllerPosePub);
    m_controller_left_pose_publisher[sn] = this->create_publisher<geometry_msgs::msg::PoseStamped>(controllerLeftPose.c_str(), 10);
    std::string controllerRightPose = "xv_sdk/SN" + sn +"/rightControllerPose";
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rightControllerPosePub;
    m_controller_right_pose_publisher.emplace(sn, rightControllerPosePub);
    m_controller_right_pose_publisher[sn] = this->create_publisher<geometry_msgs::msg::PoseStamped>(controllerRightPose.c_str(), 10);

    std::string controllerLeftData = "xv_sdk/SN" + sn +"/leftControllerData";
    rclcpp::Publisher<xv_ros2_msgs::msg::Controller>::SharedPtr leftControllerDataPub;
    m_controller_left_data_publisher.emplace(sn, leftControllerDataPub);
    m_controller_left_data_publisher[sn] = this->create_publisher<xv_ros2_msgs::msg::Controller>(controllerLeftData.c_str(), 10);
    std::string controllerRightData = "xv_sdk/SN" + sn +"/rightControllerData";
    rclcpp::Publisher<xv_ros2_msgs::msg::Controller>::SharedPtr rightControllerDataPub;
    m_controller_right_data_publisher.emplace(sn, rightControllerDataPub);
    m_controller_right_data_publisher[sn] = this->create_publisher<xv_ros2_msgs::msg::Controller>(controllerRightData.c_str(), 10);

    std::cout << "start controller port " << this->getFrameID("controller_port") << std::endl;
    m_controllerMap[sn]->startController(this->getFrameID("controller_port"));
    // auto controller_start_service_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::ControllerStart::Request> req,
    //                                     std::shared_ptr<xv_ros2_msgs::srv::ControllerStart::Response> res) -> bool
    // {
    //     std::cout << "port address: " << req->portaddress << std::endl;
    //     bool success = m_controllerMap[sn]->startController(req->portaddress);
    //     if(success)
    //     {
    //         res->success = true;
    //         res->message = "start controller ok";
    //     }
    //     else
    //     {
    //         res->success = false;
    //         res->message = "could not start controller";
    //     }
    //     return success;
    // };
    // std::string controllerStart = "xv_sdk/SN" + sn +"/controller_start";
    // rclcpp::Service<xv_ros2_msgs::srv::ControllerStart>::SharedPtr startControllerSrv;
    // m_controller_start.emplace(sn, startControllerSrv);
    // m_controller_start[sn] = this->create_service<xv_ros2_msgs::srv::ControllerStart>(controllerStart.c_str(), controller_start_service_callback);

    // auto controller_stop_service_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::ControllerStop::Request> /*req*/,
    //                                     std::shared_ptr<xv_ros2_msgs::srv::ControllerStop::Response> res) -> bool
    // {
    //     bool success = m_controllerMap[sn]->stopController();
    //     if(success)
    //     {
    //         res->success = true;
    //         res->message = "stop controller ok";
    //     }
    //     else
    //     {
    //         res->success = false;
    //         res->message = "could not stop controller";
    //     }
    //     return success;
    // };
    // std::string controllerStop = "xv_sdk/SN" + sn +"/controller_stop";
    // rclcpp::Service<xv_ros2_msgs::srv::ControllerStop>::SharedPtr stopControllerSrv;
    // m_controller_stop.emplace(sn, stopControllerSrv);
    // m_controller_stop[sn] = this->create_service<xv_ros2_msgs::srv::ControllerStop>(controllerStop.c_str(), controller_stop_service_callback);
}

void xvision_ros2_node::initFrameIDParameter(void)
{
    this->declare_parameter<std::string>("map_optical_frame",
                                         "map_optical_frame");
    this->declare_parameter<std::string>("imu_optical_frame",
                                        "imu_optical_frame");
    this->declare_parameter<std::string>("imu_optical_frame_unfiltered",
                                         "imu_optical_frame_unfiltered");
    this->declare_parameter<std::string>("fisheye_left_optical_frame",
                                         "fisheye_left_optical_frame");
    this->declare_parameter<std::string>("fisheye_right_optical_frame",
                                         "fisheye_right_optical_frame");
    this->declare_parameter<std::string>("color_optical_frame",
                                         "color_optical_frame");
    this->declare_parameter<std::string>("sgbm_optical_frame",
                                         "sgbm_optical_frame");
    this->declare_parameter<std::string>("tof_optical_frame",
                                         "tof_optical_frame");
    this->declare_parameter<std::string>("imu_link",
                                         "imu_link");
    this->declare_parameter<std::string>("base_link",
                                         "base_link");
    this->declare_parameter<std::string>("odom",
                                         "odom");
    this->declare_parameter<std::float_t>("imu_linear_acceleration",
                                         0.0);
    this->declare_parameter<std::float_t>("imu_angular_velocity",
                                         0.0);
    // this->declare_parameter<std::true_type>("slam_path_enable",
    //                                     false);
}

void xvision_ros2_node::get_frameId_parameters(void)
{
    std::string frmaId;
    std::string paramName;
    rclcpp::Parameter frmaId_param;
    std::vector<std::string> paramNames{"map_optical_frame",
                                        "imu_optical_frame",
                                        "imu_optical_frame_unfiltered",
                                        "fisheye_left_optical_frame",
                                        "fisheye_right_optical_frame",
                                        "color_optical_frame",
                                        "sgbm_optical_frame",
                                        "tof_optical_frame",
                                        "imu_link",
                                        "base_link",
                                        "controller_port"};
    for(auto &param_name : paramNames)
    {
        bool success = this->get_parameter_or(param_name, frmaId_param, rclcpp::Parameter(param_name, param_name));
        if(!success)
        {
            printInfoMsg("get parameter is fail" + param_name);
            continue;
        }
        printInfoMsg(frmaId_param.value_to_string());
        m_frameID[param_name] = frmaId_param.value_to_string();
    }
}

void xvision_ros2_node::get_device_config_parameters(void)
{
    m_deviceConfig["slam_path_enable"] = false;
    m_deviceConfig["slam_pose_enable"] = true;
    m_deviceConfig["fisheye_enable"] = false;
    bool configState = false;
    for(auto &config_name : m_deviceConfig)
    {
        bool success = this->get_parameter_or(config_name.first, configState, config_name.second);
        if(!success)
        {
            printErrorMsg("Launch file has no " + config_name.first + " parameter");
            continue;
        }
        config_name.second = configState;
    }
    for(auto &config : m_deviceConfig)
    {
        std::cout<<config.first<< ":"<< config.second<<std::endl;
    }
}

void xvision_ros2_node::printInfoMsg(const std::string msgString) const
{
    RCLCPP_INFO(this->get_logger(), "xv_sdk: '%s'", msgString.c_str());
}

void xvision_ros2_node::printErrorMsg(const std::string msgString) const
{
    RCLCPP_ERROR(this->get_logger(), "xv_sdk: '%s'", msgString.c_str());
}

void xvision_ros2_node::publishImu(std::string sn, const sensor_msgs::msg::Imu &imuMsg)
{
    if (m_imuPublisher[sn])
    {
        m_imuPublisher[sn]->publish(imuMsg);
    }
}

void xvision_ros2_node::publisheOrientation(std::string sn, const xv_ros2_msgs::msg::OrientationStamped& orientationMsg)
{
    if(m_orientationPublisher[sn])
    {
        m_orientationPublisher[sn]->publish(orientationMsg);
    }
}

void xvision_ros2_node::publishFEImage(std::string sn,
                                       const rosImage& image,
                                       const rosCamInfo& cameraInfo,
                                       xv_dev_wrapper::FE_IMAGE_TYPE imageType)
{
    if(imageType == xv_dev_wrapper::FE_IMAGE_TYPE::LEFT_IMAGE)
    {
        if(m_fisheyeImageLeftPublisher[sn] && m_fisheyeImageLeftInfoPublisher[sn])
        {
            m_fisheyeImageLeftInfoPublisher[sn]->publish(cameraInfo);
            m_fisheyeImageLeftPublisher[sn].publish(image);
        }
    }
    else if(imageType == xv_dev_wrapper::FE_IMAGE_TYPE::RIGHT_IMAGE)
    {
        if(m_fisheyeImageRightPublisher[sn] && m_fisheyeImageRightInfoPublisher[sn])
        {
            m_fisheyeImageRightInfoPublisher[sn]->publish(cameraInfo);
            m_fisheyeImageRightPublisher[sn].publish(image);
        }
    }
}

void xvision_ros2_node::publishFEAntiDistortionImage(std::string sn,
                                       const rosImage& image,
                                       const rosCamInfo& cameraInfo,
                                       xv_dev_wrapper::FE_IMAGE_TYPE imageType)
{
    if(imageType == xv_dev_wrapper::FE_IMAGE_TYPE::LEFT_IMAGE)
    {
        if(m_fisheyeAntiDistortionLeftImagePublisher[sn] && m_fisheyeAntiDistortionCamLeftInfoPublisher[sn])
        {
            m_fisheyeAntiDistortionCamLeftInfoPublisher[sn]->publish(cameraInfo);
            m_fisheyeAntiDistortionLeftImagePublisher[sn].publish(image);
        }
    }
    else if(imageType == xv_dev_wrapper::FE_IMAGE_TYPE::RIGHT_IMAGE)
    {
        if(m_fisheyeAntiDistortionRightImagePublisher[sn] && m_fisheyeAntiDistortionCamRightInfoPublisher[sn])
        {
            m_fisheyeAntiDistortionCamRightInfoPublisher[sn]->publish(cameraInfo);
            m_fisheyeAntiDistortionRightImagePublisher[sn].publish(image);
        }
    }
}

void xvision_ros2_node::publishSlamPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose)
{
    if(m_slam_pose_publisher[sn])
    {
        m_slam_pose_publisher[sn]->publish(pose);
    }
}

void xvision_ros2_node::publishSlamTrajectory(std::string sn, const nav_msgs::msg::Path& path)
{
    if(m_slam_path_publisher[sn])
    {
        m_slam_path_publisher[sn]->publish(path);
    }
}

void xvision_ros2_node::publishTofCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_tof_camera_publisher[sn].publish(image);//, cameraInfo);
    m_tof_cameraInfo_pub[sn]->publish(cameraInfo);
}

void xvision_ros2_node::publishRGBCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_rgb_camera_publisher[sn].publish(image);//, cameraInfo);
    m_rgb_cameraInfo_pub[sn]->publish(cameraInfo);
}

void xvision_ros2_node::publishSGBMImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_sgbm_camera_publisher[sn].publish(image);//, cameraInfo);
    m_sgbm_cameraInfo_pub[sn]->publish(cameraInfo);
}

void xvision_ros2_node::publishSGBMRawImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_sgbm_raw_camera_publisher[sn].publish(image);//, cameraInfo);
    m_sgbm_raw_cameraInfo_pub[sn]->publish(cameraInfo);
}

void xvision_ros2_node::publishRGBDCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_rgbd_camera_publisher[sn].publish(image);//, cameraInfo);
    m_rgbd_cameraInfo_pub[sn]->publish(cameraInfo);
}

void xvision_ros2_node::publishRGBDRawCameraImage(std::string sn, const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_rgbd_raw_camera_publisher[sn].publish(image);//, cameraInfo);
    m_rgbd_raw_cameraInfo_pub[sn]->publish(cameraInfo);
}

void xvision_ros2_node::publishRGBDRawCameraData(std::string sn, const xv_ros2_msgs::msg::ColorDepth& colorDephData)
{
    if(m_rgbd_raw_data_publisher[sn])
    {
        m_rgbd_raw_data_publisher[sn]->publish(colorDephData);
    }
}

void xvision_ros2_node::publishEvent(std::string sn, const xv_ros2_msgs::msg::EventData& eventMsg)
{
    if(m_eventPublisher[sn])
    {
        m_eventPublisher[sn]->publish(eventMsg);
    }
}

std::string xvision_ros2_node::getFrameID(const std::string &defaultId)
{
    std::string framId;
    this->get_parameter(defaultId, framId);
    return framId;
}

bool xvision_ros2_node::getConfig(std::string configName)
{
    return m_deviceConfig[configName];
}

void xvision_ros2_node::broadcasterTfTransform(const geometry_msgs::msg::TransformStamped& transform)
{
    if(m_static_tf_broadcaster)
    {
        m_static_tf_broadcaster->sendTransform(transform);
    }
}

void xvision_ros2_node::watchDevices(void)
{
    while (!m_stopWatchDevices)
    {
        xv::setLogLevel(xv::LogLevel::info);
        std::map<std::string, std::shared_ptr<Device>> device_map;
        std::string json = "";
        std::string jsonPath = "/etc/xvisio/config.json";
        std::ifstream ifs(jsonPath);
        if (ifs.is_open())
        {
            std::stringstream fbuf;
            fbuf << ifs.rdbuf();
            json = fbuf.str();
            ifs.close();
            device_map = getDevices(.0, json);
        }
        else
        {
            device_map = getDevices(.0);
        }

        for(auto &pair : device_map)
        {
            m_devSerialNumber = pair.first;
            if(m_serialNumbers.empty())
            {
                std::cout << "find new device " << m_devSerialNumber << std::endl;
                m_serialNumbers.push_back(m_devSerialNumber);
                m_deviceMap.emplace(m_devSerialNumber, std::make_shared<xv_dev_wrapper>(this, device_map[m_devSerialNumber], m_devSerialNumber, 1));
                std::cout << "init device topic" << std::endl;
                initTopicAndServer(m_devSerialNumber);
            }
            else
            {
                auto it = find(m_serialNumbers.begin(), m_serialNumbers.end(), m_devSerialNumber);
                if(it == m_serialNumbers.end())
                {
                    std::cout << "find new device " << m_devSerialNumber << std::endl;
                    m_serialNumbers.push_back(m_devSerialNumber);
                    m_deviceMap.emplace(m_devSerialNumber, std::make_shared<xv_dev_wrapper>(this, device_map[m_devSerialNumber], m_devSerialNumber, 1));
                    std::cout << "init device topic" << std::endl;
                    initTopicAndServer(m_devSerialNumber);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void xvision_ros2_node::watchControllers(void)
{
    while (!m_stopWatchDevices)
    {
        xv::setLogLevel(xv::LogLevel::info);
        std::map<std::string, std::shared_ptr<Device>> controller_map;
        std::string json = "";
        std::string jsonPath = "/etc/xvisio/config.json";
        std::ifstream ifs(jsonPath);
        if (ifs.is_open())
        {
            std::stringstream fbuf;
            fbuf << ifs.rdbuf();
            json = fbuf.str();
            ifs.close();
            controller_map = getDevices(.0, json, nullptr, xv::SlamStartMode::Normal, xv::DeviceSupport::ONLY_WIRELESS_CONTROLLER);
        }
        else
        {
            controller_map = getDevices(.0, "", nullptr, xv::SlamStartMode::Normal, xv::DeviceSupport::ONLY_WIRELESS_CONTROLLER);
        }

        for(auto &pair : controller_map)
        {
            // m_controllerSerialNumber = pair.first;
            if(m_controllerSerials.empty())
            {
                std::cout << "new controller device" << std::endl;
                m_controllerSerials.push_back(m_controllerSerialNumber);
                m_controllerMap.emplace(m_controllerSerialNumber, std::make_shared<xv_dev_wrapper>(this, controller_map[pair.first], m_controllerSerialNumber, 2));
                initControllerTopicAndServer(m_controllerSerialNumber);
            }
            else
            {
                auto it = find(m_controllerSerials.begin(), m_controllerSerials.end(), m_controllerSerialNumber);
                if(it == m_controllerSerials.end())
                {
                    std::cout << "new controller device" << std::endl;
                    m_controllerSerials.push_back(m_controllerSerialNumber);
                    m_controllerMap.emplace(m_controllerSerialNumber, std::make_shared<xv_dev_wrapper>(this, controller_map[m_controllerSerialNumber], m_controllerSerialNumber, 2));
                    initControllerTopicAndServer("Controller");
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}

void xvision_ros2_node::publishLeftControllerPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose)
{
    if(m_controller_left_pose_publisher[sn])
    {
        m_controller_left_pose_publisher[sn]->publish(pose);
    }
}

void xvision_ros2_node::publishRightControllerPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose)
{
    if(m_controller_right_pose_publisher[sn])
    {
        m_controller_right_pose_publisher[sn]->publish(pose);
    }
}

void xvision_ros2_node::publishLeftControllerData(std::string sn, const xv_ros2_msgs::msg::Controller& controllerMsg)
{
    if(m_controller_left_data_publisher[sn])
    {
        m_controller_left_data_publisher[sn]->publish(controllerMsg);
    }
}

void xvision_ros2_node::publishRightControllerData(std::string sn, const xv_ros2_msgs::msg::Controller& controllerMsg)
{
    if(m_controller_right_data_publisher[sn])
    {
        m_controller_right_data_publisher[sn]->publish(controllerMsg);
    }
}

void xvision_ros2_node::publishButton(std::string sn, int buttonType, const xv_ros2_msgs::msg::ButtonMsg& buttonMsg)
{
    switch(buttonType)
    {
        case 1:
            if(m_buttonPublisher1[sn])
            {
                m_buttonPublisher1[sn]->publish(buttonMsg);
            }
            break;
        case 2:
            if(m_buttonPublisher2[sn])
            {
                m_buttonPublisher2[sn]->publish(buttonMsg);
            }
            break;
    }
}