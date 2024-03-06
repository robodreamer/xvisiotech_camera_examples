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
    initTopicAndServer();

    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    m_watchDevThread = std::thread(&xvision_ros2_node::watchDevices, this);
}

void xvision_ros2_node::initTopicAndServer(void)
{
    m_imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    m_orientationPublisher = this->create_publisher<xv_ros2_msgs::msg::OrientationStamped>("orientation",1);
    auto imuSensor_startOri_callBack =[this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool ret = this->m_device ? this->m_device->startImuOri() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_imuSensor_startOri = this->create_service<std_srvs::srv::Trigger>("start_orientation", imuSensor_startOri_callBack);
    
    auto imuSensor_stopOri_callBack =[this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool
    {
        bool ret =  this->m_device ? this->m_device->stopImuOri() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_imuSensor_stopOri = this->create_service<std_srvs::srv::Trigger>("stop_orientation", imuSensor_stopOri_callBack);
    
    auto imuSensor_getOri_callback = [this](const std::shared_ptr<xv_ros2_msgs::srv::GetOrientation_Request> req,
                                          std::shared_ptr<xv_ros2_msgs::srv::GetOrientation_Response> res) -> bool
    {
        return this->m_device ? this->m_device->getImuOri(res->orientation, req->prediction) : false;
    };
    m_service_imuSensor_getOri = this->create_service<xv_ros2_msgs::srv::GetOrientation>("get_orientation", imuSensor_getOri_callback);
    
    auto imuSensor_getOriAt_callback = [this](const std::shared_ptr<xv_ros2_msgs::srv::GetOrientationAt_Request> req,
                                    std::shared_ptr<xv_ros2_msgs::srv::GetOrientationAt_Response> res) -> bool
    {
        return this->m_device ? this->m_device->getImuOriAt(res->orientation, req->timestamp) : false;
    };
    m_service_imuSensor_getOriAt = this->create_service<xv_ros2_msgs::srv::GetOrientationAt>("get_orientation_at", imuSensor_getOriAt_callback);

    m_fisheyeImagePublisher[0] = image_transport::create_publisher(
        this, "fisheye_cameras_left/image");//this->create_publisher<sensor_msgs::msg::Image>("", 10);
    m_fisheyeCamInfoPublisher[0] = this->create_publisher<sensor_msgs::msg::CameraInfo>("fisheye_cameras_left/camera_info", 1);
    
    m_fisheyeImagePublisher[1] = image_transport::create_publisher(
        this, "fisheye_cameras_right/image");//this->create_publisher<sensor_msgs::msg::Image>("fisheye_cameras_right/image", 10);
    m_fisheyeCamInfoPublisher[1] = this->create_publisher<sensor_msgs::msg::CameraInfo>("fisheye_cameras_right/camera_info", 1);

    m_fisheyeAntiDistortionImagePublisher[0] = image_transport::create_publisher(
        this, "fisheye_AntiDistortion_cameras_left/image");//this->create_publisher<sensor_msgs::msg::Image>("", 10);
    m_fisheyeAntiDistortionCamInfoPublisher[0] = this->create_publisher<sensor_msgs::msg::CameraInfo>("fisheye_AntiDistortion_cameras_left/camera_info", 1);
    
    m_fisheyeAntiDistortionImagePublisher[1] = image_transport::create_publisher(
        this, "fisheye_AntiDistortion_cameras_right/image");//this->create_publisher<sensor_msgs::msg::Image>("fisheye_cameras_right/image", 10);
    m_fisheyeAntiDistortionCamInfoPublisher[1] = this->create_publisher<sensor_msgs::msg::CameraInfo>("fisheye_AntiDistortion_cameras_right/camera_info", 1);
    
    auto slam_start_service_callback = [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->start_slam() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_slam_start = this->create_service<std_srvs::srv::Trigger>("start_slam", slam_start_service_callback);

    auto slam_stop_service_callback = [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool 
    {
        bool ret = this->m_device ?this->m_device->stop_slam() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_slam_stop = this->create_service<std_srvs::srv::Trigger>("stop_slam", slam_stop_service_callback);

    auto slam_get_pose_service_callback = [this](const std::shared_ptr<xv_ros2_msgs::srv::GetPose::Request> req,
                                        std::shared_ptr<xv_ros2_msgs::srv::GetPose::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->slam_get_pose(res->pose, req->prediction) : false;
        return ret;
    };
    m_service_slam_get_pose = this->create_service<xv_ros2_msgs::srv::GetPose>("get_pose", slam_get_pose_service_callback);

    auto slam_get_pose_at_service_callback = [this](const std::shared_ptr<xv_ros2_msgs::srv::GetPoseAt::Request> req,
                                        std::shared_ptr<xv_ros2_msgs::srv::GetPoseAt::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->slam_get_pose_at(res->pose, req->timestamp) : false;
        return ret;
    };
    m_service_slam_get_pose_at = this->create_service<xv_ros2_msgs::srv::GetPoseAt>("xv_sdk/service/get_pose_at", slam_get_pose_at_service_callback);
    
    if(m_deviceConfig["slam_pose_enable"])
    {
        m_slam_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    }

    if(m_deviceConfig["slam_path_enable"])
    {
        m_slam_path_publisher = this->create_publisher<nav_msgs::msg::Path>("trajectory", 1);
    }

    auto tof_start_service_callback = [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->start_tof() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_tof_start = this->create_service<std_srvs::srv::Trigger>("start_tof", tof_start_service_callback);

    auto tof_stop_service_callback = [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->stop_tof() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_tof_stop = this->create_service<std_srvs::srv::Trigger>("stop_tof", tof_stop_service_callback);
    
    m_tof_camera_publisher = image_transport::create_publisher(
        this, "tof/depth/image_rect_raw");//image_transport::create_camera_publisher(this, "camera/depth/image_rect_raw");
    m_tof_cameraInfo_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "tof/depth/camera_info", 1);

    auto rgb_start_service_callback = [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->start_rgb() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_rgb_start = this->create_service<std_srvs::srv::Trigger>("start_rgb", rgb_start_service_callback);

    auto rgb_stop_service_callback = [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> bool 
    {
        bool ret = this->m_device ? this->m_device->stop_rgb() : false;
        res->success = ret;
        res->message = ret ? "successed" : "failed";
        return ret;
    };
    m_service_rgb_stop = this->create_service<std_srvs::srv::Trigger>("stop_rgb", rgb_stop_service_callback);
    
    m_rgb_camera_publisher = image_transport::create_publisher(
        this, "rgb/image");
    m_rgb_cameraInfo_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 1);   

    m_sgbm_camera_publisher = image_transport::create_publisher(
        this, "xv_sdk/sgbm/image");
    m_sgbm_cameraInfo_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("xv_sdk/sgbm/camera_info", 1);   
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
                                        "base_link"};
    for(auto &param_name : paramNames)
    {
        bool ret = this->get_parameter_or(param_name, frmaId_param, rclcpp::Parameter(param_name, param_name));
        if(!ret)
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
        bool ret = this->get_parameter_or(config_name.first, configState, config_name.second);
        if(!ret)
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

void xvision_ros2_node::publishImu(const sensor_msgs::msg::Imu &imuMsg)
{
    if (m_imuPublisher)
    {
        m_imuPublisher->publish(imuMsg);
    }
}

void xvision_ros2_node::publisheOrientation(const xv_ros2_msgs::msg::OrientationStamped& orientationMsg)
{
    if(m_orientationPublisher)
    {
        m_orientationPublisher->publish(orientationMsg);
    }
}

void xvision_ros2_node::publishFEImage(const rosImage& image,
                                       const rosCamInfo& cameraInfo,
                                       xv_dev_wrapper::FE_IMAGE_TYPE imageType)
{
    if(m_fisheyeImagePublisher[imageType] && m_fisheyeCamInfoPublisher[imageType])
    {
        m_fisheyeCamInfoPublisher[imageType]->publish(cameraInfo);
        m_fisheyeImagePublisher[imageType].publish(image);
    }
}

void xvision_ros2_node::publishFEAntiDistortionImage(const rosImage& image,
                                       const rosCamInfo& cameraInfo,
                                       xv_dev_wrapper::FE_IMAGE_TYPE imageType)
{
    if(m_fisheyeAntiDistortionImagePublisher[imageType] && m_fisheyeAntiDistortionCamInfoPublisher[imageType])
    {
        m_fisheyeAntiDistortionCamInfoPublisher[imageType]->publish(cameraInfo);
        m_fisheyeAntiDistortionImagePublisher[imageType].publish(image);
    }
}

void xvision_ros2_node::publishSlamPose(const geometry_msgs::msg::PoseStamped& pose)
{
    if(m_slam_pose_publisher)
    {
        m_slam_pose_publisher->publish(pose);
    }
}

void xvision_ros2_node::publishSlamTrajectory(const nav_msgs::msg::Path& path)
{
    if(m_slam_path_publisher)
    {
        m_slam_path_publisher->publish(path);
    }
}

void xvision_ros2_node::publishTofCameraImage(const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_tof_camera_publisher.publish(image);//, cameraInfo);
    m_tof_cameraInfo_pub->publish(cameraInfo);
}

void xvision_ros2_node::publishRGBCameraImage(const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_rgb_camera_publisher.publish(image);//, cameraInfo);
    m_rgb_cameraInfo_pub->publish(cameraInfo);
}

void xvision_ros2_node::publishSGBMImage(const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& cameraInfo)
{
    m_sgbm_camera_publisher.publish(image);//, cameraInfo);
    m_sgbm_cameraInfo_pub->publish(cameraInfo);
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
            device_map = getDevices(0., json);
        }
        else
        {
            device_map = getDevices(0.);
        }

        std::vector<std::string> serialNumbers;
        for (auto &pair : device_map)
        {
            serialNumbers.emplace_back(pair.first);
        }

        for (const std::string &serialNumber : serialNumbers)
        {
            auto it = m_deviceMap.find(serialNumber);
            if (it == m_deviceMap.end()) 
            {
                m_device = std::make_shared<xv_dev_wrapper>(this, device_map[serialNumber]);
                m_deviceMap.emplace(serialNumber, m_device);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}