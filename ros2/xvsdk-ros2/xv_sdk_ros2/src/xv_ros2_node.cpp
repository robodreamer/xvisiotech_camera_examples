#include "xv_ros2_node.h"
xvision_ros2_node::xvision_ros2_node(void) :
    Node("xvisio_SDK", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)),
    count_(0),
    m_stopWatchDevices(false)
{
    // Create the remapped pose publisher
    m_remapped_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/xv_sdk/slam/pose", 10);
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
    // IMU publisher
    std::string imuPublisherTopic = "xv_sdk/SN" + sn +"/imu";
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    m_imuPublisher.emplace(sn, imuPublisher);
    m_imuPublisher[sn] = this->create_publisher<sensor_msgs::msg::Imu>(imuPublisherTopic.c_str(), 10);

    // Only create orientation publisher if enabled
    if (m_deviceConfig["orientation_enable"]) {
        std::string orientationPublisherTopic = "xv_sdk/SN" + sn +"/orientation";
        rclcpp::Publisher<xv_ros2_msgs::msg::OrientationStamped>::SharedPtr orientationPublisher;
        m_orientationPublisher.emplace(sn, orientationPublisher);
        m_orientationPublisher[sn] = this->create_publisher<xv_ros2_msgs::msg::OrientationStamped>(orientationPublisherTopic.c_str(),1);
    }

    // Start orientation service
    auto imuSensor_startOrientation_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool success = m_deviceMap[sn] ? m_deviceMap[sn]->startImuOri() : false;
        res->success = success;
        res->message = success ? "successed" : "failed";
        return success;
    };
    std::string startOrientationService = "xv_sdk/SN" + sn +"/start_orientation";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startOrientationServiceHandle;
    m_service_imuSensor_startOri.emplace(sn, startOrientationServiceHandle);
    m_service_imuSensor_startOri[sn] = this->create_service<std_srvs::srv::Trigger>(startOrientationService.c_str(), imuSensor_startOrientation_callback);

    // Stop orientation service
    auto imuSensor_stopOrientation_callback = [this, sn](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool success =  m_deviceMap[sn] ? m_deviceMap[sn]->stopImuOri() : false;
        res->success = success;
        res->message = success ? "successed" : "failed";
        return success;
    };
    std::string stopOrientationService = "xv_sdk/SN" + sn +"/stop_orientation";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopOrientationServiceHandle;
    m_service_imuSensor_stopOri.emplace(sn, stopOrientationServiceHandle);
    m_service_imuSensor_stopOri[sn] = this->create_service<std_srvs::srv::Trigger>(stopOrientationService.c_str(), imuSensor_stopOrientation_callback);

    // Get orientation service
    auto imuSensor_getOrientation_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::GetOrientation_Request> req,
                                          std::shared_ptr<xv_ros2_msgs::srv::GetOrientation_Response> res) -> bool
    {
        return m_deviceMap[sn] ? m_deviceMap[sn]->getImuOri(res->orientation, req->prediction) : false;
    };
    std::string getOrientationService = "xv_sdk/SN" + sn +"/get_orientation";
    rclcpp::Service<xv_ros2_msgs::srv::GetOrientation>::SharedPtr getOrientationServiceHandle;
    m_service_imuSensor_getOri.emplace(sn, getOrientationServiceHandle);
    m_service_imuSensor_getOri[sn] = this->create_service<xv_ros2_msgs::srv::GetOrientation>(getOrientationService.c_str(), imuSensor_getOrientation_callback);

    // Get orientation at time service
    auto imuSensor_getOrientationAt_callback = [this, sn](const std::shared_ptr<xv_ros2_msgs::srv::GetOrientationAt_Request> req,
                                    std::shared_ptr<xv_ros2_msgs::srv::GetOrientationAt_Response> res) -> bool
    {
        return m_deviceMap[sn] ? m_deviceMap[sn]->getImuOriAt(res->orientation, req->timestamp) : false;
    };
    std::string getOrientationAtService = "xv_sdk/SN" + sn +"/get_orientation_at";
    rclcpp::Service<xv_ros2_msgs::srv::GetOrientationAt>::SharedPtr getOrientationAtServiceHandle;
    m_service_imuSensor_getOriAt.emplace(sn, getOrientationAtServiceHandle);
    m_service_imuSensor_getOriAt[sn] = this->create_service<xv_ros2_msgs::srv::GetOrientationAt>(getOrientationAtService.c_str(), imuSensor_getOrientationAt_callback);

    // Only create fisheye publishers if enabled
    if (m_deviceConfig["fisheye_enable"]) {
        std::string feImage0 = "xv_sdk/SN" + sn +"/fisheye_cameras_left/image";
        image_transport::Publisher feImageLeftPub;
        m_fisheyeImageLeftPublisher.emplace(sn, feImageLeftPub);
        m_fisheyeImageLeftPublisher[sn] = image_transport::create_publisher(
            this, feImage0.c_str());
        std::string feInfo0 = "xv_sdk/SN" + sn +"/fisheye_cameras_left/camera_info";
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feImageLeftInfoPub;
        m_fisheyeImageLeftInfoPublisher.emplace(sn, feImageLeftInfoPub);
        m_fisheyeImageLeftInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfo0.c_str(), 1);

        std::string feImage1 = "xv_sdk/SN" + sn +"/fisheye_cameras_right/image";
        image_transport::Publisher feImageRightPub;
        m_fisheyeImageRightPublisher.emplace(sn, feImageRightPub);
        m_fisheyeImageRightPublisher[sn] = image_transport::create_publisher(
            this, feImage1.c_str());
        std::string feInfo1 = "xv_sdk/SN" + sn +"/fisheye_cameras_right/camera_info";
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feImagRightInfoPub;
        m_fisheyeImageRightInfoPublisher.emplace(sn, feImagRightInfoPub);
        m_fisheyeImageRightInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfo1.c_str(), 1);

        std::string feImageAnti0 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_left/image";
        image_transport::Publisher feAntiImageLeftPub;
        m_fisheyeAntiDistortionLeftImagePublisher.emplace(sn, feAntiImageLeftPub);
        m_fisheyeAntiDistortionLeftImagePublisher[sn] = image_transport::create_publisher(
            this, feImageAnti0.c_str());
        std::string feInfoAnti0 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_left/camera_info";
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feAntiImageLeftInfoPub;
        m_fisheyeAntiDistortionCamLeftInfoPublisher.emplace(sn, feAntiImageLeftInfoPub);
        m_fisheyeAntiDistortionCamLeftInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfoAnti0.c_str(), 1);

        std::string feImageAnti1 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_right/image";
        image_transport::Publisher feAntiImageRightPub;
        m_fisheyeAntiDistortionRightImagePublisher.emplace(sn, feAntiImageRightPub);
        m_fisheyeAntiDistortionRightImagePublisher[sn] = image_transport::create_publisher(
            this, feImageAnti1.c_str());
        std::string feInfoAnti1 = "xv_sdk/SN" + sn +"/fisheye_AntiDistortion_cameras_right/camera_info";
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr feAntiImageRightInfoPub;
        m_fisheyeAntiDistortionCamRightInfoPublisher.emplace(sn, feAntiImageRightInfoPub);
        m_fisheyeAntiDistortionCamRightInfoPublisher[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(feInfoAnti1.c_str(), 1);
    }

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

    // Only create trajectory publisher if slam_path_enable is true
    if(m_deviceConfig["slam_path_enable"])
    {
        std::string trajectoryTopic = "xv_sdk/SN" + sn +"/trajectory";
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slamPathPublisher;
        m_slam_path_publisher.emplace(sn, slamPathPublisher);
        m_slam_path_publisher[sn] = this->create_publisher<nav_msgs::msg::Path>(trajectoryTopic.c_str(), 1);
    }

    // Only create ToF publishers if enabled
    if (m_deviceConfig["tof_enable"]) {
        std::string tofImage = "xv_sdk/SN" + sn +"/tof/depth/image_rect_raw";
        image_transport::Publisher tofImagePub;
        m_tof_camera_publisher.emplace(sn, tofImagePub);
        m_tof_camera_publisher[sn] = image_transport::create_publisher(
            this, tofImage.c_str());
        std::string tofInfo = "xv_sdk/SN" + sn +"/tof/depth/camera_info";
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr tofImageInfo;
        m_tof_cameraInfo_pub.emplace(sn, tofImageInfo);
        m_tof_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            tofInfo.c_str(), 1);
    }

    // Only create RGB publishers if enabled
    if (m_deviceConfig["rgb_enable"]) {
        std::string rgbImage = "xv_sdk/SN" + sn +"/rgb/image";
        image_transport::Publisher rgbImagePub;
        m_rgb_camera_publisher.emplace(sn, rgbImagePub);
        m_rgb_camera_publisher[sn] = image_transport::create_publisher(
            this, rgbImage.c_str());
        std::string rgbInfo = "xv_sdk/SN" + sn +"/rgb/camera_info";
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbImageInfoPub;
        m_rgb_cameraInfo_pub.emplace(sn, rgbImageInfoPub);
        m_rgb_cameraInfo_pub[sn] = this->create_publisher<sensor_msgs::msg::CameraInfo>(rgbInfo.c_str(), 1);
    }

    // Only create RGBD publishers if enabled
    if (m_deviceConfig["rgbd_enable"]) {
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
    }

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

    // Only create event publisher if enabled
    if (m_deviceConfig["event_enable"]) {
        std::string eventPubName = "xv_sdk/SN" + sn +"/event";
        rclcpp::Publisher<xv_ros2_msgs::msg::EventData>::SharedPtr eventPub;
        m_eventPublisher.emplace(sn, eventPub);
        m_eventPublisher[sn] = this->create_publisher<xv_ros2_msgs::msg::EventData>(eventPubName.c_str(), 1);
    }

    // Only create button publishers if enabled
    if (m_deviceConfig["button_states_enable"]) {
        std::string buttonName1 = "xv_sdk/SN" + sn +"/button1";
        rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr buttonPublisher1;
        m_buttonPublisher1.emplace(sn, buttonPublisher1);
        m_buttonPublisher1[sn] = this->create_publisher<xv_ros2_msgs::msg::ButtonMsg>(buttonName1.c_str(),1);

        std::string buttonName2 = "xv_sdk/SN" + sn +"/button2";
        rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr buttonPublisher2;
        m_buttonPublisher2.emplace(sn, buttonPublisher2);
        m_buttonPublisher2[sn] = this->create_publisher<xv_ros2_msgs::msg::ButtonMsg>(buttonName2.c_str(),1);

        std::string buttonName3 = "xv_sdk/SN" + sn +"/button3";
        rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr buttonPublisher3;
        m_buttonPublisher3.emplace(sn, buttonPublisher3);
        m_buttonPublisher3[sn] = this->create_publisher<xv_ros2_msgs::msg::ButtonMsg>(buttonName3.c_str(),1);

        std::string buttonName4 = "xv_sdk/SN" + sn +"/button4";
        rclcpp::Publisher<xv_ros2_msgs::msg::ButtonMsg>::SharedPtr buttonPublisher4;
        m_buttonPublisher4.emplace(sn, buttonPublisher4);
        m_buttonPublisher4[sn] = this->create_publisher<xv_ros2_msgs::msg::ButtonMsg>(buttonName4.c_str(),1);
    }
}

void xvision_ros2_node::initControllerTopicAndServer(std::string sn)
{
    RCLCPP_INFO(this->get_logger(), "Initializing controller topics for %s", sn.c_str());

    // Create controller publishers
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

    RCLCPP_INFO(this->get_logger(), "Starting controller on port %s", this->getFrameID("controller_port").c_str());
    m_controllerMap[sn]->startController(this->getFrameID("controller_port"));
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
    std::vector<std::string> config_params = {
        "slam_path_enable", "slam_pose_enable", "fisheye_enable",
        "button_states_enable", "rgb_enable", "tof_enable", "rgbd_enable",
        "event_enable", "orientation_enable"
    };

    for (const auto& param : config_params) {
        bool value = false;
        this->get_parameter(param, value);
        m_deviceConfig[param] = value;
    }

    // Log all parameters
    for (const auto& config : m_deviceConfig) {
        RCLCPP_INFO(this->get_logger(), "%s: %s", config.first.c_str(), config.second ? "true" : "false");
    }
}

void xvision_ros2_node::printInfoMsg(const std::string msgString) const
{
    RCLCPP_INFO(this->get_logger(), "xv_sdk: '%s'", msgString.c_str());
}

void xvision_ros2_node::printWarningMsg(const std::string msgString) const
{
    RCLCPP_WARN(this->get_logger(), "xv_sdk: '%s'", msgString.c_str());
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
    // Only publish if orientation is enabled
    if (!m_deviceConfig["orientation_enable"]) {
        return;
    }

    if (m_orientationPublisher.count(sn) > 0) {
        m_orientationPublisher[sn]->publish(orientationMsg);
    }
}

void xvision_ros2_node::publishFEImage(std::string sn,
                                       const rosImage& image,
                                       const rosCamInfo& cameraInfo,
                                       xv_dev_wrapper::FE_IMAGE_TYPE imageType)
{
    // Only publish if fisheye is enabled
    if (!m_deviceConfig["fisheye_enable"]) {
        return;
    }

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
    // Only publish if fisheye is enabled
    if (!m_deviceConfig["fisheye_enable"]) {
        return;
    }

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

        // Also publish to the remapped topic
        if(m_remapped_pose_publisher)
        {
            m_remapped_pose_publisher->publish(pose);
        }
    }
}

void xvision_ros2_node::publishSlamTrajectory(std::string sn, const nav_msgs::msg::Path& path)
{
    // Only publish if slam_path is enabled
    if (!m_deviceConfig["slam_path_enable"]) {
        return;
    }

    if (m_slam_path_publisher.count(sn) > 0) {
        m_slam_path_publisher[sn]->publish(path);
    }
}

void xvision_ros2_node::publishTofCameraImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    // Only publish if ToF is enabled
    if (!m_deviceConfig["tof_enable"]) {
        return;
    }

    if (m_tof_camera_publisher.count(sn) > 0 && m_tof_cameraInfo_pub.count(sn) > 0) {
        m_tof_camera_publisher[sn].publish(image);
        m_tof_cameraInfo_pub[sn]->publish(cameraInfo);
    }
}

void xvision_ros2_node::publishRGBCameraImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    // Only publish if RGB is enabled
    if (!m_deviceConfig["rgb_enable"]) {
        return;
    }

    if (m_rgb_camera_publisher.count(sn) > 0 && m_rgb_cameraInfo_pub.count(sn) > 0) {
        m_rgb_camera_publisher[sn].publish(image);
        m_rgb_cameraInfo_pub[sn]->publish(cameraInfo);
    }
}

void xvision_ros2_node::publishRGBDCameraImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    // Only publish if RGBD is enabled
    if (!m_deviceConfig["rgbd_enable"]) {
        return;
    }

    if (m_rgbd_camera_publisher.count(sn) > 0 && m_rgbd_cameraInfo_pub.count(sn) > 0) {
        m_rgbd_camera_publisher[sn].publish(image);
        m_rgbd_cameraInfo_pub[sn]->publish(cameraInfo);
    }
}

void xvision_ros2_node::publishRGBDRawCameraImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    // Only publish if RGBD is enabled
    if (!m_deviceConfig["rgbd_enable"]) {
        return;
    }

    if (m_rgbd_raw_camera_publisher.count(sn) > 0 && m_rgbd_raw_cameraInfo_pub.count(sn) > 0) {
        m_rgbd_raw_camera_publisher[sn].publish(image);
        m_rgbd_raw_cameraInfo_pub[sn]->publish(cameraInfo);
    }
}

void xvision_ros2_node::publishRGBDRawCameraData(std::string sn, const xv_ros2_msgs::msg::ColorDepth& colorDephData)
{
    // Only publish if RGBD is enabled
    if (!m_deviceConfig["rgbd_enable"]) {
        return;
    }

    if (m_rgbd_raw_data_publisher.count(sn) > 0) {
        m_rgbd_raw_data_publisher[sn]->publish(colorDephData);
    }
}

void xvision_ros2_node::publishEvent(std::string sn, const xv_ros2_msgs::msg::EventData& eventMsg)
{
    // Only publish if event is enabled
    if (!m_deviceConfig["event_enable"]) {
        return;
    }

    if (m_eventPublisher.count(sn) > 0) {
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

void xvision_ros2_node::broadcasterStaticTfTransform(const geometry_msgs::msg::TransformStamped& transform)
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
                RCLCPP_INFO(this->get_logger(), "find new device %s", m_devSerialNumber.c_str());
                m_serialNumbers.push_back(m_devSerialNumber);
                m_deviceMap.emplace(m_devSerialNumber, std::make_shared<xv_dev_wrapper>(this, device_map[m_devSerialNumber], m_devSerialNumber, 1));
                RCLCPP_INFO(this->get_logger(), "init device topic");
                initTopicAndServer(m_devSerialNumber);
            }
            else
            {
                auto it = find(m_serialNumbers.begin(), m_serialNumbers.end(), m_devSerialNumber);
                if(it == m_serialNumbers.end())
                {
                    RCLCPP_INFO(this->get_logger(), "find new device %s", m_devSerialNumber.c_str());
                    m_serialNumbers.push_back(m_devSerialNumber);
                    m_deviceMap.emplace(m_devSerialNumber, std::make_shared<xv_dev_wrapper>(this, device_map[m_devSerialNumber], m_devSerialNumber, 1));
                    RCLCPP_INFO(this->get_logger(), "init device topic");
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

        // Only proceed if controllers are found
        if (!controller_map.empty()) {
            for(auto &pair : controller_map)
            {
                if(m_controllerSerials.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "New controller device detected: %s", m_controllerSerialNumber.c_str());
                    m_controllerSerials.push_back(m_controllerSerialNumber);
                    m_controllerMap.emplace(m_controllerSerialNumber, std::make_shared<xv_dev_wrapper>(this, controller_map[pair.first], m_controllerSerialNumber, 2));
                    // Start the controller before creating publishers
                    if (m_controllerMap[m_controllerSerialNumber]->startController(this->getFrameID("controller_port"))) {
                        initControllerTopicAndServer(m_controllerSerialNumber);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to start controller: %s", m_controllerSerialNumber.c_str());
                    }
                }
                else
                {
                    auto it = find(m_controllerSerials.begin(), m_controllerSerials.end(), m_controllerSerialNumber);
                    if(it == m_controllerSerials.end())
                    {
                        RCLCPP_INFO(this->get_logger(), "New controller device detected: %s", m_controllerSerialNumber.c_str());
                        m_controllerSerials.push_back(m_controllerSerialNumber);
                        m_controllerMap.emplace(m_controllerSerialNumber, std::make_shared<xv_dev_wrapper>(this, controller_map[pair.first], m_controllerSerialNumber, 2));
                        // Start the controller before creating publishers
                        if (m_controllerMap[m_controllerSerialNumber]->startController(this->getFrameID("controller_port"))) {
                            initControllerTopicAndServer(m_controllerSerialNumber);
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Failed to start controller: %s", m_controllerSerialNumber.c_str());
                        }
                    }
                }
            }
        } else {
            // If no controllers found, clean up existing controller resources
            if (!m_controllerSerials.empty()) {
                RCLCPP_INFO(this->get_logger(), "Controllers disconnected, cleaning up resources");
                for (const auto& sn : m_controllerSerials) {
                    if (m_controllerMap.count(sn) > 0) {
                        m_controllerMap[sn]->stopController();
                        m_controllerMap.erase(sn);
                    }
                }
                m_controllerSerials.clear();

                // Clear all controller publishers
                m_controller_left_pose_publisher.clear();
                m_controller_right_pose_publisher.clear();
                m_controller_left_data_publisher.clear();
                m_controller_right_data_publisher.clear();
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

        // Also publish to the remapped topic if no SLAM pose is being published
        if(m_remapped_pose_publisher && m_serialNumbers.empty())
        {
            m_remapped_pose_publisher->publish(pose);
        }
    }
}

void xvision_ros2_node::publishRightControllerPose(std::string sn, const geometry_msgs::msg::PoseStamped& pose)
{
    if(m_controller_right_pose_publisher[sn])
    {
        m_controller_right_pose_publisher[sn]->publish(pose);

        // Also publish to the remapped topic if no SLAM pose is being published
        if(m_remapped_pose_publisher && m_serialNumbers.empty())
        {
            m_remapped_pose_publisher->publish(pose);
        }
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
    // Only publish if button states are enabled
    if (!m_deviceConfig["button_states_enable"]) {
        return;
    }

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
        case 3:
            if(m_buttonPublisher3[sn])
            {
                m_buttonPublisher3[sn]->publish(buttonMsg);
            }
            break;
        case 4:
            if(m_buttonPublisher4[sn])
            {
                m_buttonPublisher4[sn]->publish(buttonMsg);
            }
            break;
    }
}

bool xvision_ros2_node::isFramePublished(const std::string& frame_id)
{
    return m_published_frames.find(frame_id) != m_published_frames.end();
}

void xvision_ros2_node::setFramePublished(const std::string& frame_id)
{
    m_published_frames.insert(frame_id);
}

void xvision_ros2_node::publishSGBMImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    // Only publish if fisheye is enabled (since SGBM depends on fisheye)
    if (!m_deviceConfig["fisheye_enable"]) {
        return;
    }

    if (m_sgbm_camera_publisher.count(sn) > 0 && m_sgbm_cameraInfo_pub.count(sn) > 0) {
        m_sgbm_camera_publisher[sn].publish(image);
        m_sgbm_cameraInfo_pub[sn]->publish(cameraInfo);
    }
}

void xvision_ros2_node::publishSGBMRawImage(std::string sn, const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    // Only publish if fisheye is enabled (since SGBM depends on fisheye)
    if (!m_deviceConfig["fisheye_enable"]) {
        return;
    }

    if (m_sgbm_raw_camera_publisher.count(sn) > 0 && m_sgbm_raw_cameraInfo_pub.count(sn) > 0) {
        m_sgbm_raw_camera_publisher[sn].publish(image);
        m_sgbm_raw_cameraInfo_pub[sn]->publish(cameraInfo);
    }
}
