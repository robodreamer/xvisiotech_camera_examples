#include "xv_dev_wrapper.h"
#include "xv_ros2_node.h"
#include <cstring>
#include <chrono>

xv_dev_wrapper::xv_dev_wrapper(xvision_ros2_node *node,
                               std::shared_ptr<xv::Device> device,
                               std::string sn,
                               int type):
    m_node(node),
    m_device(device),
    m_sn(sn),
    m_type(type)
{
    m_slam_pose_enable = m_node->getConfig("slam_pose_enable");
    m_slam_path_enable = m_node->getConfig("slam_path_enable");
    m_fisheye_enable = m_node->getConfig("fisheye_enable");
    init();
}

void xv_dev_wrapper::init(void)
{
    if(m_type == 1)
    {
        if (m_device->imuSensor())
        {
            initImu();
        }

        if(m_device->orientationStream() && m_node->getConfig("orientation_enable"))
        {
            initOrientationStream();
        }

        if(m_device->fisheyeCameras())
        {
            m_fisheye_cameras = m_device->fisheyeCameras();
            initFisheyeCameras();
        }

        //TODO detect plane
        if (m_device->slam())
        {
            initSlam();
        }

        if(m_device->tofCamera() && m_node->getConfig("tof_enable"))
        {
            initTofCamera();
        }

        if(m_device->colorCamera() && m_node->getConfig("rgb_enable"))
        {
            initColorCamera();
        }

        if(m_device->colorCamera() && m_device->tofCamera() && m_node->getConfig("rgbd_enable"))
        {
            initColorDepthCamera();
        }

        if(m_device->eventStream() && (m_node->getConfig("event_enable") || m_node->getConfig("button_states_enable")))
        {
            initEvent();
        }
    }
}

bool xv_dev_wrapper::startImuOri()
{
    return m_device->orientationStream()->start();
}

bool xv_dev_wrapper::stopImuOri(void)
{
    return m_device->orientationStream()->stop();
}

bool xv_dev_wrapper::getImuOri(xv_ros2_msgs::msg::OrientationStamped& oriStamped, const builtin_interfaces::msg::Duration& duration)
{
    Orientation ori;
    bool ok = m_device->orientationStream()->get(ori, duration.sec);
    if (ok)
    {
        formatXvOriToRosOriStamped(oriStamped ,ori, m_node->getFrameID("map_optical_frame"));
    }

    return ok;
}

bool xv_dev_wrapper::getImuOriAt(xv_ros2_msgs::msg::OrientationStamped& oriStamped, const builtin_interfaces::msg::Time &time)
{
    Orientation ori;
    bool ok = m_device->orientationStream()->getAt(ori, time.sec);
    if (ok)
    {
        formatXvOriToRosOriStamped(oriStamped ,ori, m_node->getFrameID("map_optical_frame"));
    }

    return ok;
}

bool xv_dev_wrapper::start_slam(void)
{
    bool success = true;
    if(this->m_device->slam())
    {
        success = this->m_device->slam()->start();
    }
    else
    {
        success = false;
        this->m_node->printErrorMsg("slam is not available");
    }
    return success;
}

bool xv_dev_wrapper::stop_slam(void)
{
    bool success = true;
    if(this->m_device->slam())
    {
        this->m_device->slam()->stop();
    }
    else
    {
        success = false;
        this->m_node->printErrorMsg("slam is not available");
    }
    return success;
}

bool xv_dev_wrapper::slam_get_pose(geometry_msgs::msg::PoseStamped& poseSteamped, const builtin_interfaces::msg::Duration& prediction)
{
    xv::Pose pose;
    bool ok = this->m_device->slam()->getPose(pose, get_sec(prediction));
    if (ok)
    {
        poseSteamped = to_ros_poseStamped(pose, this->m_node->getFrameID("map_optical_frame"));
    }
    return ok;
}

bool xv_dev_wrapper::slam_get_pose_at(geometry_msgs::msg::PoseStamped& poseSteamped, const builtin_interfaces::msg::Time& time)
{
    Pose pose;
    bool ok = m_device->slam()->getPoseAt(pose, get_sec(time));
    if (ok)
    {
        poseSteamped = to_ros_poseStamped(pose, this->m_node->getFrameID("map_optical_frame"));
    }

    return ok;
}

bool xv_dev_wrapper::start_tof(void)
{
  return m_device ? m_device->tofCamera()->start() : false;
}

bool xv_dev_wrapper::stop_tof(void)
{
    return m_device ? m_device->tofCamera()->stop() : false;
}

bool xv_dev_wrapper::start_rgb(void)
{
  return m_device ? m_device->colorCamera()->start() : false;
}

bool xv_dev_wrapper::stop_rgb(void)
{
    return m_device ? m_device->colorCamera()->stop() : false;
}

void xv_dev_wrapper::initImu(void)
{
    this->m_node->printInfoMsg(std::string("start IMU"));
    m_device->imuSensor()->start();
    auto imuCallbackFun = [this](const Imu &xvImu) {
        sensor_msgs::msg::Imu rosImu;
        formatImuTopicMsg(rosImu, xvImu);
        this->m_node->publishImu(m_sn, rosImu);
    };
    m_device->imuSensor()->registerCallback(imuCallbackFun);
}

void xv_dev_wrapper::initOrientationStream(void)
{
    auto orientationStreamCallbackFun = [this](const Orientation & xvOrientation)
    {
        if (xvOrientation.hostTimestamp < 0)
        {
            this->m_node->printErrorMsg(std::string("XVSDK-ROS-WRAPPER toRosOrientationStamped() Error: negative Orientation host-timestamp"));
        }
        xv_ros2_msgs::msg::OrientationStamped orientationStamped;
        toRosOrientationStamped(orientationStamped, xvOrientation, this->m_node->getFrameID("map_optical_frame"));
        this->m_node->publisheOrientation(m_sn, orientationStamped);
    };
    m_device->orientationStream()->registerCallback(orientationStreamCallbackFun);
}

void xv_dev_wrapper::initFisheyeCameras(void)
{
    getFECalibration();
    m_fisheye_cameras->start();
    registerFECallbackFunc();
    // registerSGBMCallbackFunc();
}

void xv_dev_wrapper::initSlam(void)
{
    auto slam_reister_callback = [this](const Pose& pose)
    {
        if (pose.hostTimestamp() < 0)
        {
            this->m_node->printErrorMsg("slam negative Pose host-timestamp");
            return;
        }
        if(m_slam_pose_enable)
        {
            geometry_msgs::msg::PoseStamped poseSteamped = to_ros_poseStamped(pose, this->m_node->getFrameID("map_optical_frame"));
            this->m_node->publishSlamPose(m_sn, poseSteamped);
            this->m_node->broadcasterTfTransform(toRosTransformStamped(pose,
                                                           this->m_node->getFrameID("map_optical_frame"),
                                                           this->m_node->getFrameID("imu_optical_frame")));
        }

        if(m_slam_path_enable)
        {
            this->m_path_msgs.header.stamp = get_stamp_from_sec(steady_clock_now());
            this->m_path_msgs.header.frame_id = this->m_node->getFrameID("odom");
            this->m_node->publishSlamTrajectory(m_sn, toRosPoseStampedRetNavmsgs(pose, this->m_node->getFrameID("odom"), this->m_path_msgs));
            this->m_node->broadcasterTfTransform(toRosTransformStamped(pose,
                                                           this->m_node->getFrameID("base_link"),
                                                           this->m_node->getFrameID("odom")));
        }
    };

    m_device->slam()->registerCallback(slam_reister_callback);
    this->m_node->printInfoMsg("slam start");
    m_device->slam()->start();
}

void xv_dev_wrapper::initTofCamera()
{
    if (!m_device->tofCamera()->calibration().empty())
    {
        m_xvTofCalib = m_device->tofCamera()->calibration()[0];
    }
    else
    {
        this->m_node->printErrorMsg("tof calibration is empty");
    }
    m_tofCameraInfo = toRosCameraInfo(m_xvTofCalib.ucm.empty()?nullptr:&m_xvTofCalib.ucm[0],
                                      m_xvTofCalib.pdcm.empty()?nullptr:&m_xvTofCalib.pdcm[0]);
    m_device->tofCamera()->registerCallback([this](const DepthImage & xvDepthImage)
    {
        if (!xvDepthImage.data)
        {
            this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: no DepthImage data");
            return;
        }
        if (xvDepthImage.hostTimestamp < 0)
        {
            this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: negative DepthImage host-timestamp");
            return;
        }

        // ignore IR
        if (xvDepthImage.type != xv::DepthImage::Type::Depth_16 && xvDepthImage.type != xv::DepthImage::Type::Depth_32) {
            return;
        }

        m_tof_mutex.lock();
        m_xvDepthImage = xvDepthImage;
        xv_ros2_msgs::msg::ColorDepth colorDephData;
        toRosColorDepthData(colorDephData, this->m_node->getFrameID("rgbd_optical_frame"));
        this->m_node->publishRGBDRawCameraData(m_sn, colorDephData);
        m_tof_mutex.unlock();

        sensor_msgs::msg::Image img = toRosImage(xvDepthImage, this->m_node->getFrameID("tof_optical_frame"));
        this->m_tofCameraInfo.header.frame_id = img.header.frame_id;
        this->m_tofCameraInfo.header.stamp = img.header.stamp;

        this->m_node->publishTofCameraImage(m_sn, img, m_tofCameraInfo);
    });
    this->m_node->printInfoMsg("tof start");
    this->m_device->tofCamera()->start();
}

void xv_dev_wrapper::initColorCamera()
{
    if (!m_device->colorCamera()->calibration().empty())
    {
        m_xvRGBCalib = m_device->colorCamera()->calibration()[0];
    }
    else
    {
        this->m_node->printErrorMsg("RGB calibration is empty");
    }
    m_rgbCameraInfo = toRosCameraInfo(m_xvRGBCalib.ucm.empty()?nullptr:&m_xvRGBCalib.ucm[0],
                                      m_xvRGBCalib.pdcm.empty()?nullptr:&m_xvRGBCalib.pdcm[0]);
    m_device->colorCamera()->registerCallback([this](const ColorImage & xvColorImage)
    {
        if (!xvColorImage.data)
        {
            this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: no rgb data");
            return;
        }
        if (xvColorImage.hostTimestamp < 0)
        {
            this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: negative rgb host-timestamp");
            return;
        }

        m_rgb_mutex.lock();
        m_xvColorImage = xvColorImage;
        m_rgb_mutex.unlock();

        sensor_msgs::msg::Image img = toRosImage(xvColorImage, this->m_node->getFrameID("rgb_optical_frame"));
        this->m_rgbCameraInfo.header.frame_id = img.header.frame_id;
        this->m_rgbCameraInfo.header.stamp = img.header.stamp;
        this->m_node->publishRGBCameraImage(m_sn, img, m_rgbCameraInfo);
    });
    this->m_node->printInfoMsg("rgb start");
    this->m_device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
    this->m_device->colorCamera()->start();
}

void xv_dev_wrapper::initColorDepthCamera()
{
    if (!m_device->tofCamera()->calibration().empty())
    {
        m_xvRGBDCalib = m_device->tofCamera()->calibration()[0];
    }
    else
    {
        this->m_node->printErrorMsg("RGBD calibration is empty");
    }
    m_rgbdCameraInfo = toRosCameraInfo(m_xvRGBDCalib.ucm.empty()?nullptr:&m_xvRGBDCalib.ucm[0],
                                      m_xvRGBDCalib.pdcm.empty()?nullptr:&m_xvRGBDCalib.pdcm[0]);
    m_rgbdRawCameraInfo = toRosCameraInfo(m_xvRGBDCalib.ucm.empty()?nullptr:&m_xvRGBDCalib.ucm[0],
                                      m_xvRGBDCalib.pdcm.empty()?nullptr:&m_xvRGBDCalib.pdcm[0]);
    m_device->tofCamera()->registerColorDepthImageCallback([this](const DepthColorImage & xvDepthColorImage)
    {
        if (!xvDepthColorImage.data)
        {
            this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: no rgbd data");
            return;
        }
        if (xvDepthColorImage.hostTimestamp < 0)
        {
            this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: negative rgbd host-timestamp");
            return;
        }

        sensor_msgs::msg::Image img = toRosImage(xvDepthColorImage, this->m_node->getFrameID("rgbd_optical_frame"));
        this->m_rgbdCameraInfo.header.frame_id = img.header.frame_id;
        this->m_rgbdCameraInfo.header.stamp = img.header.stamp;
        this->m_node->publishRGBDCameraImage(m_sn, img, m_rgbdCameraInfo);

        sensor_msgs::msg::Image imgRaw = toRosImageRaw(xvDepthColorImage, this->m_node->getFrameID("rgbd_optical_frame"));
        this->m_rgbdRawCameraInfo.header.frame_id = img.header.frame_id;
        this->m_rgbdRawCameraInfo.header.stamp = img.header.stamp;
        this->m_node->publishRGBDRawCameraImage(m_sn, imgRaw, m_rgbdRawCameraInfo);

    });
    this->m_node->printInfoMsg("rgbd start");
}

void xv_dev_wrapper::initEvent()
{
    m_device->eventStream()->start();
    m_device->eventStream()->registerCallback([this](const xv::Event & event)
    {
        xv_ros2_msgs::msg::EventData eventMsg;
        toRosEventStamped(eventMsg, event, this->m_node->getFrameID("imu_optical_frame"));
        this->m_node->publishEvent(m_sn, eventMsg);

        xv_ros2_msgs::msg::ButtonMsg buttonMsg;
        toRosButtonStamped(buttonMsg, event, this->m_node->getFrameID("imu_optical_frame"));
        this->m_node->publishButton(m_sn, (int)event.type, buttonMsg);
    });
}

void xv_dev_wrapper::toRosEventStamped(xv_ros2_msgs::msg::EventData& event, xv::Event const& xvEvent, const std::string& frame_id)
{
    if (xvEvent.hostTimestamp < 0) {
        this->printWarningMsg("XVSDK-ROS-WRAPPER toRosEventStamped() Warning: negative host-timestamp: " + std::to_string(xvEvent.hostTimestamp));
    }

    event.header.stamp = get_stamp_from_sec(xvEvent.hostTimestamp > 0.1 ? xvEvent.hostTimestamp : 0.1);
    event.header.frame_id = frame_id;

    event.type = xvEvent.type;
    event.state = xvEvent.state;
}

void xv_dev_wrapper::toRosButtonStamped(xv_ros2_msgs::msg::ButtonMsg& button, xv::Event const& xvEvent, const std::string& frame_id)
{
    if (xvEvent.hostTimestamp < 0)
        this->printWarningMsg("XVSDK-ROS-WRAPPER toRosButtonStamped() Warning: negative host-timestamp: " + std::to_string(xvEvent.hostTimestamp));

    button.header.stamp = get_stamp_from_sec(xvEvent.hostTimestamp > 0.1 ? xvEvent.hostTimestamp : 0.1);
    button.header.frame_id = frame_id;
    button.state = (bool)xvEvent.state;
}

sensor_msgs::msg::CameraInfo xv_dev_wrapper::toRosCameraInfo(const xv::UnifiedCameraModel* const ucm,
                                                             const xv::PolynomialDistortionCameraModel* const pdcm)
{
    sensor_msgs::msg::CameraInfo camInfo;
    if (pdcm)
    {
        this->m_node->printInfoMsg("use pdcm calibration");
        /// Most ROS users will prefer PDM if available
        const auto& c = *pdcm;
        camInfo.height = c.h;
        camInfo.width = c.w;
        camInfo.distortion_model = "plumb_bob"; /// XXX is that correct ?
        camInfo.d = {c.distor.begin(), c.distor.end()};
        camInfo.k = {c.fx,    0, c.u0,
                    0, c.fy, c.v0,
                    0,    0,    1};
        camInfo.binning_x = 1;
        camInfo.binning_y = 1;
    }
    else if (ucm)
    {
        this->m_node->printInfoMsg("use ucm calibration");
        const auto& c = *ucm;
        camInfo.height = c.h;
        camInfo.width = c.w;
        camInfo.distortion_model = "unified"; /// XXX is that correct ?
        camInfo.d = {c.xi};
        camInfo.k = {c.fx,    0, c.u0,
                    0, c.fy, c.v0,
                    0,    0,    1};
        camInfo.binning_x = 1;
        camInfo.binning_y = 1;
    }

    return camInfo;
}

sensor_msgs::msg::CameraInfo xv_dev_wrapper::toRosCameraInfo(const xv::SpecialUnifiedCameraModel* const seucm)
{
    sensor_msgs::msg::CameraInfo camInfo;
    if (seucm)
    {
        this->m_node->printInfoMsg("use seucm calibration");
        /// Most ROS users will prefer PDM if available
        const auto& c = *seucm;
        camInfo.height = c.h;
        camInfo.width = c.w;
        camInfo.distortion_model = "plumb_bob"; /// XXX is that correct ?
        camInfo.d = {c.eu, c.ev};
        camInfo.k = {c.fx,    0, c.u0,
                    0, c.fy, c.v0,
                    0,    0,    1};
        camInfo.binning_x = 1;
        camInfo.binning_y = 1;
    }

    return camInfo;
}

void xv_dev_wrapper::formatImuTopicMsg(sensor_msgs::msg::Imu &rosImu, const xv::Imu &xvImu)
{
    rosImu.header.stamp = getHeaderStamp(xvImu.hostTimestamp);
    rosImu.header.frame_id = m_node->getFrameID("imu_optical_frame");
    rosImu.orientation_covariance = {
        -1, -1, -1, -1, -1, -1, -1, -1, -1}; /// no orientation data, row major
    rosImu.angular_velocity.x = xvImu.gyro[0];
    rosImu.angular_velocity.y = xvImu.gyro[1];
    rosImu.angular_velocity.z = xvImu.gyro[2];
    const double avcov = 0.0; // m_param_imuSensor_angular_velocity_stddev*m_param_imuSensor_angular_velocity_stddev;
                            // /// mutex ?
    rosImu.angular_velocity_covariance = {avcov, 0, 0, 0, avcov,
                                          0, 0, 0, avcov}; /// row major
    rosImu.linear_acceleration.x = xvImu.accel[0];
    rosImu.linear_acceleration.y = xvImu.accel[1];
    rosImu.linear_acceleration.z = xvImu.accel[2];
    const double lacov = 0.0; // m_param_imuSensor_linear_acceleration_stddev*m_param_imuSensor_linear_acceleration_stddev;
                            // /// mutex ?
    rosImu.linear_acceleration_covariance = {lacov, 0, 0, 0, lacov,
                                             0, 0, 0, lacov}; /// row major
}

void xv_dev_wrapper::formatXvOriToRosOriStamped(xv_ros2_msgs::msg::OrientationStamped &rosOrientation,
                                                const xv::Orientation& xvOrientation,
                                                const std::string& frameId )
{
    rosOrientation.header.stamp = getHeaderStamp(xvOrientation.hostTimestamp);
    rosOrientation.header.frame_id = frameId;//this->m_node->getFrameID("map_optical_frame");

    for (int i = 0; i < 9; ++i)
        rosOrientation.matrix[i] = xvOrientation.rotation()[i];

    auto quat = xvOrientation.quaternion(); /// [qx,qy,qz,qw]
    rosOrientation.quaternion.x = quat[0];
    rosOrientation.quaternion.y = quat[1];
    rosOrientation.quaternion.z = quat[2];
    rosOrientation.quaternion.w = quat[3];

    rosOrientation.angular_velocity.x = xvOrientation.angularVelocity()[0];
    rosOrientation.angular_velocity.y = xvOrientation.angularVelocity()[1];
    rosOrientation.angular_velocity.z = xvOrientation.angularVelocity()[2];
}

void xv_dev_wrapper::registerFECallbackFunc(void)
{
    this->m_node->printInfoMsg("fisheye start");
    m_fisheye_cameras->registerCallback([this](const FisheyeImages & xvFisheyeImages)
    {
        for (int i = 0; i < int(xvFisheyeImages.images.size()); ++i)
        {
            const auto& xvGrayImage = xvFisheyeImages.images[i];

            if (!xvGrayImage.data)
            {
                this->m_node->printWarningMsg("XVSDK-ROS-WRAPPER Warning: no FisheyeImages data");
                return;
            }
            if (xvFisheyeImages.hostTimestamp < 0)
            {
                this->m_node->printWarningMsg("XVSDK-ROS-WRAPPER Warning: negative FisheyeImages host-timestamp");
                return;
            }

            auto img = changeFEGrayScaleImage2RosImage(xvGrayImage, xvFisheyeImages.hostTimestamp, "");

            if (i == 0)
            {
                img.header.frame_id = this->m_node->getFrameID("fisheye_left_optical_frame");

                sensor_msgs::msg::CameraInfo camInfo = m_fisheyeCameraInfos[i][img.height];
                camInfo.header.frame_id = img.header.frame_id;
                camInfo.header.stamp = img.header.stamp;
                this->m_node->publishFEImage(m_sn, img, camInfo, LEFT_IMAGE);
            }

            if (i == 1)
            {
                img.header.frame_id = this->m_node->getFrameID("fisheye_right_optical_frame");

                sensor_msgs::msg::CameraInfo camInfo = m_fisheyeCameraInfos[i][img.height];
                camInfo.header.frame_id = img.header.frame_id;
                camInfo.header.stamp = img.header.stamp;
                this->m_node->publishFEImage(m_sn, img, camInfo, RIGHT_IMAGE);
            }
        }
    });
}


static struct xv::sgbm_config global_config = {
    1 ,//enable_dewarp
    1.0, //dewarp_zoom_factor
    0, //enable_disparity
    1, //enable_depth
    0, //enable_point_cloud
    0.08, //baseline
    96, //fov
    255, //disparity_confidence_threshold
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
    1, //enable_gamma
    2.2, //gamma_value
    0, //enable_gaussian
    0, //mode
    8000, //max_distance
    100, //min_distance
};

void xv_dev_wrapper::registerSGBMCallbackFunc(void)
{
    m_device->sgbmCamera()->start(global_config);
    if (!m_device->fisheyeCameras()->calibration().empty())
    {
        m_xvSgbmCalib = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(m_device->fisheyeCameras())->calibrationEx()[0]; //temp
    }
    else
    {
        this->m_node->printErrorMsg("fisheye calibration is empty");
    }
    m_sgbmCamInfo = toRosCameraInfo(m_xvSgbmCalib.seucm.empty()?nullptr:&m_xvSgbmCalib.seucm[0]);
    m_device->sgbmCamera()->registerCallback([this](const xv::SgbmImage& xvSgbmImage){
        if(xvSgbmImage.type == xvSgbmImage.Type::Depth)
        {
            if (!xvSgbmImage.data)
            {
                this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: no SgbmImage data");
                return;
            }

            if (xvSgbmImage.hostTimestamp < 0)
            {
                this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Warning: negative SgbmImage host-timestamp");
                return;
            }

            auto img = toRosImage(xvSgbmImage, this->m_node->getFrameID("sgbm_frame"));
            auto imgRaw = sgbmRawDepthtoRosImage(xvSgbmImage, this->m_node->getFrameID("sgbm_raw_frame"));

            this->m_sgbmCamInfo.header.frame_id = img.header.frame_id;
            this->m_sgbmCamInfo.header.stamp = img.header.stamp;

            this->m_node->publishSGBMImage(m_sn, img, m_sgbmCamInfo);
            this->m_node->publishSGBMRawImage(m_sn, imgRaw, m_sgbmCamInfo);
        }
    });
}

sensor_msgs::msg::Image xv_dev_wrapper::changeFEGrayScaleImage2RosImage(const GrayScaleImage& xvGrayImage, double timestamp, const std::string& frame_id)
{
    if (timestamp < 0)
    {
        m_node->printInfoMsg("XVSDK-ROS-WRAPPER toRosImage() Error: negative timestamp");
    }

    sensor_msgs::msg::Image rosImage;
    rosImage.header.stamp = get_stamp_from_sec(timestamp);
    rosImage.header.frame_id = frame_id;
    rosImage.height = xvGrayImage.height;
    rosImage.width = xvGrayImage.width;
    rosImage.encoding = sensor_msgs::image_encodings::MONO8;
    rosImage.is_bigendian = false;
    rosImage.step = xvGrayImage.width * sizeof(uint8_t);
    int nsize = xvGrayImage.width * xvGrayImage.height;
    rosImage.data = std::vector<unsigned char>(xvGrayImage.data.get(), xvGrayImage.data.get() + nsize);

    return rosImage;
}

void xv_dev_wrapper::getFECalibration()
{
    m_xvFisheyesCalibs = m_device->fisheyeCameras()->calibration();
    m_fisheyeCameraInfos.resize(m_xvFisheyesCalibs.size());
    for (int i = 0; i < int(m_xvFisheyesCalibs.size()); ++i)
    {
        const auto& calibs = m_xvFisheyesCalibs[i];
        for (const auto& calib : calibs.pdcm)
        {
            m_fisheyeCameraInfos[i][calib.h] = toRosCameraInfo(nullptr, &calib);
        }
        for (const auto& calib : calibs.ucm)
        {
            if (m_fisheyeCameraInfos[i].find(calib.h) == m_fisheyeCameraInfos[i].end())
            {
                m_fisheyeCameraInfos[i][calib.h] = toRosCameraInfo(&calib, nullptr);
            }
        }

    //   const sensor_msgs::CameraInfo& camInfo = m_fisheyeCameraInfos[i][400];

    //   const Pose ext(calibs.pose.translation(), calibs.pose.rotation(), steady_clock_now());

    //   if (i == 0)
    //   {
    //     m_camInfoMan_fisheye_left->setCameraInfo(camInfo);
    //     s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
    //                                                                getFrameId("imu_optical_frame"),
    //                                                                getFrameId("fisheye_left_optical_frame")));
    //   }

    //   if (i == 1)
    //   {
    //     m_camInfoMan_fisheye_right->setCameraInfo(camInfo);
    //     s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
    //                                                                getFrameId("imu_optical_frame"),
    //                                                                getFrameId("fisheye_right_optical_frame")));
    //   }
    }
}

double xv_dev_wrapper::get_sec(const builtin_interfaces::msg::Duration& prediction) const
{
     return (double)prediction.sec + 1e-9*(double)prediction.nanosec;
}

double xv_dev_wrapper::get_sec(const builtin_interfaces::msg::Time& timestamp)const
{
    return (double)timestamp.sec + 1e-9*(double)timestamp.nanosec;
}

geometry_msgs::msg::PoseStamped xv_dev_wrapper::to_ros_poseStamped(const Pose& xvPose, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped ps;
    static double old_timeStamp = steady_clock_now();
    if (xvPose.hostTimestamp() < 0)
    {
        this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosPoseStamped() Error: negative Pose host-timestamp");
    }
    try
    {
        double currentTimestamp = xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1;
        ps.header.stamp = get_stamp_from_sec(currentTimestamp);
        old_timeStamp = currentTimestamp;
    }
    catch(std::runtime_error& ex)
    {
        ps.header.stamp = get_stamp_from_sec(old_timeStamp);
    }
    ps.header.frame_id = frame_id;

    ps.pose.position.x = xvPose.x();
    ps.pose.position.y = xvPose.y();
    ps.pose.position.z = xvPose.z();

    const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
    ps.pose.orientation.x = quat[0];
    ps.pose.orientation.y = quat[1];
    ps.pose.orientation.z = quat[2];
    ps.pose.orientation.w = quat[3];

    return ps;
}

geometry_msgs::msg::PoseStamped xv_dev_wrapper::to_ros_poseEdgeStamped(const Pose& xvPose, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped ps;
    static double old_timeStamp = steady_clock_now();
    if (xvPose.edgeTimestampUs() < 0)
    {
        this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER to_ros_poseEdgeStamped() Error: negative Pose host-timestamp");
    }
    try
    {
        double currentTimestamp = xvPose.edgeTimestampUs() > 0.1 ? xvPose.edgeTimestampUs() : 0.1;
        ps.header.stamp = get_stamp_from_microsec(currentTimestamp);
        old_timeStamp = currentTimestamp;
    }
    catch(std::runtime_error& ex)
    {
        ps.header.stamp = get_stamp_from_microsec(old_timeStamp);
    }
    ps.header.frame_id = frame_id;

    ps.pose.position.x = xvPose.x();
    ps.pose.position.y = xvPose.y();
    ps.pose.position.z = xvPose.z();

    const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
    ps.pose.orientation.x = quat[0];
    ps.pose.orientation.y = quat[1];
    ps.pose.orientation.z = quat[2];
    ps.pose.orientation.w = quat[3];

    return ps;
}

builtin_interfaces::msg::Time xv_dev_wrapper::get_stamp_from_sec(double seconds) const
{
    builtin_interfaces::msg::Time stamp;
    #ifdef HAVE_TRUNC
    stamp.sec  = (int32_t)trunc(seconds);
    #else
    if (seconds >= 0.0)
        stamp.sec = (int32_t)floor(seconds);
    else
        stamp.sec = (int32_t)floor(seconds) + 1;
    #endif
        stamp.nanosec = (int32_t)((seconds - (double)stamp.sec)*1000000000);

    return stamp;
}

builtin_interfaces::msg::Time xv_dev_wrapper::get_stamp_from_microsec(double microsec) const
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = (int32_t)(microsec/1000000);
    stamp.nanosec = microsec*1000 - stamp.sec*1000000000;
    return stamp;
}

rclcpp::Time xv_dev_wrapper::getHeaderStamp(double hostTimesStamp)
{
    return rclcpp::Time( hostTimesStamp,
    (hostTimesStamp - static_cast<uint32_t>(hostTimesStamp)) * 1000000000);
}

double xv_dev_wrapper::steady_clock_now()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-9;
}

nav_msgs::msg::Path xv_dev_wrapper::toRosPoseStampedRetNavmsgs(const Pose& xvPose, const std::string& frame_id, nav_msgs::msg::Path& path)
{
    if (xvPose.hostTimestamp() < 0)
    {
        this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosPoseStamped() Error: negative Pose host-timestamp");
    }

    geometry_msgs::msg::PoseStamped this_ps;
    this_ps.header.frame_id = frame_id;
    this_ps.header.stamp = get_stamp_from_sec(xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1);
    this_ps.pose.position.x = xvPose.x();
    this_ps.pose.position.y = xvPose.y();
    this_ps.pose.position.z = xvPose.z();

    const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
    this_ps.pose.orientation.x = quat[0];
    this_ps.pose.orientation.y = quat[1];
    this_ps.pose.orientation.z = quat[2];
    this_ps.pose.orientation.w = quat[3];

    path.poses.push_back(this_ps);

    return path;
}

geometry_msgs::msg::TransformStamped xv_dev_wrapper::toRosTransformStamped(const Pose& pose, const std::string& parent_frame_id, const std::string& frame_id)
{
    geometry_msgs::msg::TransformStamped tf;
    static double old_timeStamp = steady_clock_now();
    if (pose.hostTimestamp() < 0)
    {
        this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosTransformStamped() Error: negative Pose host-timestamp");
    }

    try
    {
      double currentTimestamp = pose.hostTimestamp() > 0.1 ? pose.hostTimestamp() : 0.1;
      tf.header.stamp = get_stamp_from_sec(currentTimestamp);
      old_timeStamp = currentTimestamp;
    }
    catch(std::runtime_error& ex)
    {
        tf.header.stamp = get_stamp_from_sec(old_timeStamp);
    }
    tf.header.frame_id = parent_frame_id;
    tf.child_frame_id = frame_id;

    tf.transform.translation.x = pose.x();
    tf.transform.translation.y = pose.y();
    tf.transform.translation.z = pose.z();

    auto quat = pose.quaternion(); /// [qx,qy,qz,qw]
    tf.transform.rotation.x = quat[0];
    tf.transform.rotation.y = quat[1];
    tf.transform.rotation.z = quat[2];
    tf.transform.rotation.w = quat[3];

    return tf;
}

sensor_msgs::msg::Image xv_dev_wrapper::toRosImage(const DepthImage& xvDepthImage, const std::string& frame_id)
{
  if (xvDepthImage.hostTimestamp < 0)
    this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosImage() Error: negative DepthImage host-timestamp");

  sensor_msgs::msg::Image rosImage;
  rosImage.header.stamp = get_stamp_from_sec(xvDepthImage.hostTimestamp > 0.1 ? xvDepthImage.hostTimestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = xvDepthImage.height;
  rosImage.width = xvDepthImage.width;
  rosImage.is_bigendian = false;
  /// TODO How to avoid copy ?
  const int nbPx = xvDepthImage.width * xvDepthImage.height;
  std::vector<uint8_t> copy(nbPx*sizeof(float)/sizeof(uint8_t));
  if (xvDepthImage.type == xv::DepthImage::Type::Depth_32) {
    // std::memcpy(&copy[0], xvDepthImage.data.get(), nbPx * sizeof(float));
    rosImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    rosImage.step = xvDepthImage.width * sizeof(float); /// bytes for 1 line
    copy = std::vector<uint8_t>(xvDepthImage.data.get(), xvDepthImage.data.get() + nbPx*sizeof(float));
  } else if (xvDepthImage.type == xv::DepthImage::Type::Depth_16) {
      // static float cov = 7.5 / 2494.0; // XXX *0.001
      // rosImage.data = std::vector<uint8_t>(fl, fl + nbPx*sizeof(float));
      rosImage.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      rosImage.step = xvDepthImage.width * sizeof(short); /// bytes for 1 line
      copy = std::vector<uint8_t>(xvDepthImage.data.get(), xvDepthImage.data.get() + nbPx*sizeof(short));
  }
  rosImage.data = std::move(copy);
  return rosImage;
}

sensor_msgs::msg::Image xv_dev_wrapper::toRosImage(const ColorImage& xvColorImage, const std::string& frame_id)
{
  if (xvColorImage.hostTimestamp < 0)
    this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosImage() Error: negative ColorImage host-timestamp");

  sensor_msgs::msg::Image rosImage;
  rosImage.header.stamp = get_stamp_from_sec(xvColorImage.hostTimestamp > 0.1 ? xvColorImage.hostTimestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = xvColorImage.height;
  rosImage.width = xvColorImage.width;
  rosImage.encoding = sensor_msgs::image_encodings::BGR8; /// FIXME why need BGR to get RGB ?
  rosImage.is_bigendian = false;
  rosImage.step = xvColorImage.width * 3 * sizeof(uint8_t); /// bytes for 1 line
  /// TODO How to avoid copy ?
  const int nbPx = xvColorImage.width * xvColorImage.height;
  std::vector<uint8_t> copy(3*nbPx);
  cv::Mat cvMat = toCvMatRGB(xvColorImage);
//   std::memcpy(&copy[0], cvMat.data, nbPx * 3 * sizeof(uint8_t));
//   /// TODO use that instead when implemented
//   //std::memcpy(&copy[0], xvColorImage.toRgb().data.get(), nbPx * 3*sizeof(uint8_t));
//   rosImage.data = std::move(copy);
  rosImage.data = std::vector<unsigned char>(cvMat.data, cvMat.data + nbPx * 3 * sizeof(uint8_t));
  return rosImage;
}

static std::tuple<int, int, int> color(double distance, double distance_min, double distance_max, double threshold)
{
    double d = std::max(distance_min, std::min(distance, distance_max));
    d = (d - distance_min) / (distance_max - distance_min);
    if (distance <= threshold || distance > distance_max)
    {
        return std::tuple<int, int, int>(0, 0, 0);
    }
    int b = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.5))), 1.0));
    int g = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.25))), 1.0));
    int r = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * d)), 1.0));
    return std::tuple<int, int, int>(r, g, b);
}


static std::shared_ptr<unsigned char> depthImage(uint16_t *data, unsigned int width, unsigned int height, double min_distance_m, double max_distance_m, bool colorize)
{
    std::shared_ptr<unsigned char> out;
    if (colorize)
    {
        out = std::shared_ptr<unsigned char>(new unsigned char[width * height * 3], std::default_delete<unsigned char[]>());
    }
    else
    {
        out = std::shared_ptr<unsigned char>(new unsigned char[width * height], std::default_delete<unsigned char[]>());
    }

    for (unsigned int i = 0; i < width * height; i++)
    {
        double distance_mm = data[i];
        if (colorize)
        {
            double distance_m = distance_mm / 1000.;

            auto c = color(distance_m, min_distance_m, max_distance_m, min_distance_m);
            out.get()[i * 3 + 0] = static_cast<unsigned char>(std::get<2>(c));
            out.get()[i * 3 + 1] = static_cast<unsigned char>(std::get<1>(c));
            out.get()[i * 3 + 2] = static_cast<unsigned char>(std::get<0>(c));
        }
        else
        {
            double max_distance_mm = max_distance_m * 1000.;
            double min_distance_mm = min_distance_m * 1000.;
            distance_mm = std::min(max_distance_mm, distance_mm);
            distance_mm = std::max(distance_mm, min_distance_mm);

            double norm = (distance_mm - min_distance_mm) / (max_distance_mm - min_distance_mm);
            auto c = 255. * norm;
            out.get()[i] = static_cast<unsigned char>(c);
        }
    }

    return out;
}

cv::Mat convDepthToMat(std::shared_ptr<const xv::SgbmImage> sgbm_image,bool _colorize_depth)
{
    static double depth_max_distance_m = 5;
    static double depth_min_distance_m = 0.1;
    uint16_t* p16 = (uint16_t*)sgbm_image->data.get();

    // cv::Mat mask;
    // cv::Mat im_gray_d = cv::Mat(cv::Size(sgbm_image->width, sgbm_image->height),  CV_16UC1, p16); //18
    // cv::inRange(im_gray_d, cv::Scalar(1), cv::Scalar(65535), mask);
    // p16 = (uint16_t *)im_gray_d.data;

    double focal_length = sgbm_image->width / (2.f * tan(/*global_config.fov*/69 / 2 / 180.f * M_PI));
    double max_distance_m = (focal_length * /*global_config.baseline*/0.11285 / 1);
    double min_distance_m = 0; //0 is considered invalid distance (0 disparity == unknown)
    max_distance_m = std::min(max_distance_m, depth_max_distance_m);
    min_distance_m = depth_min_distance_m;
    assert(max_distance_m > min_distance_m);

    static std::shared_ptr<unsigned char> tmp;
    tmp = depthImage(p16, sgbm_image->width, sgbm_image->height, min_distance_m, max_distance_m, !!_colorize_depth);
    if (_colorize_depth)
    {
        cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3, tmp.get());
        // cv::Mat roi = cv::Mat::zeros(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3);
        // im_col.copyTo(roi,mask);
        return im_col;
    }
    else
    {
        cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC1, tmp.get());
        return im_col;
    }
}

sensor_msgs::msg::Image xv_dev_wrapper::toRosImage(const SgbmImage& xvSgbmDepthImage, const std::string& frame_id)
{
  if (xvSgbmDepthImage.hostTimestamp < 0)
    this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosImage() Error: negative SgbmImage host-timestamp");

  sensor_msgs::msg::Image rosImage;

  if(xv::SgbmImage::Type::Disparity == xvSgbmDepthImage.type)
  {
    this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosImage()s Error: wrong sgbm type:Disparity");
  }
  else
  {
    rosImage.header.stamp = get_stamp_from_sec(xvSgbmDepthImage.hostTimestamp > 0.1 ? xvSgbmDepthImage.hostTimestamp : 0.1);
    rosImage.header.frame_id = frame_id;
    rosImage.height = xvSgbmDepthImage.height;
    rosImage.width = xvSgbmDepthImage.width;
    rosImage.encoding = sensor_msgs::image_encodings::BGR8; /// FIXME why need BGR to get RGB ?
    rosImage.is_bigendian = false;
    rosImage.step = rosImage.width * 3*sizeof(uint8_t); /// bytes for 1 line
    const int nbPx = rosImage.width * rosImage.height;
    std::vector<uint8_t> copy(3*nbPx);
    std::shared_ptr<const xv::SgbmImage> ptr_sgbm = std::make_shared<xv::SgbmImage>(xvSgbmDepthImage);
    cv::Mat cvMat = convDepthToMat(std::shared_ptr<const xv::SgbmImage>(ptr_sgbm),true);
    // std::memcpy(&copy[0], cvMat.data, nbPx * 3*sizeof(uint8_t));
    // rosImage.data = std::move(copy);
    rosImage.data = std::vector<uint8_t>(cvMat.data, cvMat.data + nbPx * 3*sizeof(uint8_t));
  }

  return rosImage;
}

sensor_msgs::msg::Image xv_dev_wrapper::sgbmRawDepthtoRosImage(const SgbmImage& xvSgbmDepthImage, const std::string& frame_id)
{
    if (xvSgbmDepthImage.hostTimestamp < 0)
        this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosImage() Error: negative SgbmImage host-timestamp");

    sensor_msgs::msg::Image rosImage;

    if(xv::SgbmImage::Type::Disparity == xvSgbmDepthImage.type)
    {
        this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosImage()s Error: wrong sgbm type:Disparity");
    }
    else
    {
        rosImage.header.stamp = get_stamp_from_sec(xvSgbmDepthImage.hostTimestamp > 0.1 ? xvSgbmDepthImage.hostTimestamp : 0.1);
        rosImage.header.frame_id = frame_id;

        rosImage.height = xvSgbmDepthImage.height;

        rosImage.width = xvSgbmDepthImage.width;

        rosImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        rosImage.is_bigendian = false;

        rosImage.step = rosImage.width * sizeof(uint16_t);

        const int nbPx = rosImage.width * rosImage.height;

        std::vector<uint8_t> copy(2*nbPx);
        std::shared_ptr<const xv::SgbmImage> ptr_sgbm = std::make_shared<xv::SgbmImage>(xvSgbmDepthImage);
        // std::memcpy(&copy[0], xvSgbmDepthImage.data.get(), nbPx * sizeof(uint16_t));
        // rosImage.data = std::move(copy);
        rosImage.data = std::vector<uint8_t>(xvSgbmDepthImage.data.get(), xvSgbmDepthImage.data.get() + nbPx * sizeof(uint16_t));
    }

    return rosImage;
}


static std::vector<std::vector<unsigned char>> colors = {{0,0,0},
{255,4,0}, {255,8,0}, {255,12,0}, {255,17,0}, {255,21,0}, {255,25,0}, {255,29,0},
{255,34,0}, {255,38,0}, {255,42,0}, {255,46,0}, {255,51,0}, {255,55,0}, {255,59,0},
{255,64,0}, {255,68,0}, {255,72,0}, {255,76,0}, {255,81,0}, {255,85,0}, {255,89,0},
{255,93,0}, {255,98,0}, {255,102,0}, {255,106,0}, {255,110,0}, {255,115,0}, {255,119,0},
{255,123,0}, {255,128,0}, {255,132,0}, {255,136,0}, {255,140,0}, {255,145,0}, {255,149,0},
{255,153,0}, {255,157,0}, {255,162,0}, {255,166,0}, {255,170,0}, {255,174,0}, {255,179,0},
{255,183,0}, {255,187,0}, {255,191,0}, {255,196,0}, {255,200,0}, {255,204,0}, {255,209,0},
{255,213,0}, {255,217,0}, {255,221,0}, {255,226,0}, {255,230,0}, {255,234,0}, {255,238,0},
{255,243,0}, {255,247,0}, {255,251,0}, {255,255,0}, {251,255,0}, {247,255,0}, {243,255,0},
{238,255,0}, {234,255,0}, {230,255,0}, {226,255,0}, {221,255,0}, {217,255,0}, {213,255,0},
{209,255,0}, {204,255,0}, {200,255,0}, {196,255,0}, {191,255,0}, {187,255,0}, {183,255,0},
{179,255,0}, {174,255,0}, {170,255,0}, {166,255,0}, {162,255,0}, {157,255,0}, {153,255,0},
{149,255,0}, {145,255,0}, {140,255,0}, {136,255,0}, {132,255,0}, {128,255,0}, {123,255,0},
{119,255,0}, {115,255,0}, {110,255,0}, {106,255,0}, {102,255,0}, {98,255,0}, {93,255,0},
{89,255,0}, {85,255,0}, {81,255,0}, {76,255,0}, {72,255,0}, {68,255,0}, {64,255,0},
{59,255,0}, {55,255,0}, {51,255,0}, {46,255,0}, {42,255,0}, {38,255,0}, {34,255,0},
{29,255,0}, {25,255,0}, {21,255,0}, {17,255,0}, {12,255,0}, {8,255,0}, {4,255,0},
{0,255,0}, {0,255,4}, {0,255,8}, {0,255,12}, {0,255,17}, {0,255,21}, {0,255,25},
{0,255,29}, {0,255,34}, {0,255,38}, {0,255,42}, {0,255,46}, {0,255,51}, {0,255,55},
{0,255,59}, {0,255,64}, {0,255,68}, {0,255,72}, {0,255,76}, {0,255,81}, {0,255,85},
{0,255,89}, {0,255,93}, {0,255,98}, {0,255,102}, {0,255,106}, {0,255,110}, {0,255,115},
{0,255,119}, {0,255,123}, {0,255,128}, {0,255,132}, {0,255,136}, {0,255,140}, {0,255,145},
{0,255,149}, {0,255,153}, {0,255,157}, {0,255,162}, {0,255,166}, {0,255,170}, {0,255,174},
{0,255,179}, {0,255,183}, {0,255,187}, {0,255,191}, {0,255,196}, {0,255,200}, {0,255,204},
{0,255,209}, {0,255,213}, {0,255,217}, {0,255,221}, {0,255,226}, {0,255,230}, {0,255,234},
{0,255,238}, {0,255,243}, {0,255,247}, {0,255,251}, {0,255,255}, {0,251,255}, {0,247,255},
{0,243,255}, {0,238,255}, {0,234,255}, {0,230,255}, {0,226,255}, {0,221,255}, {0,217,255},
{0,213,255}, {0,209,255}, {0,204,255}, {0,200,255}, {0,196,255}, {0,191,255}, {0,187,255},
{0,183,255}, {0,179,255}, {0,174,255}, {0,170,255}, {0,166,255}, {0,162,255}, {0,157,255},
{0,153,255}, {0,149,255}, {0,145,255}, {0,140,255}, {0,136,255}, {0,132,255}, {0,128,255},
{0,123,255}, {0,119,255}, {0,115,255}, {0,110,255}, {0,106,255}, {0,102,255}, {0,98,255},
{0,93,255}, {0,89,255}, {0,85,255}, {0,81,255}, {0,76,255}, {0,72,255}, {0,68,255},
{0,64,255}, {0,59,255}, {0,55,255}, {0,51,255}, {0,46,255}, {0,42,255}, {0,38,255},
{0,34,255}, {0,29,255}, {0,25,255}, {0,21,255}, {0,17,255}, {0,12,255}, {0,8,255},
{0,4,255}, {0,0,255}, {4,0,255}, {8,0,255}, {12,0,255}, {17,0,255}, {21,0,255},
{25,0,255}, {29,0,255}, {34,0,255}, {38,0,255}, {42,0,255}, {46,0,255}, {51,0,255},
{55,0,255}, {59,0,255}, {64,0,255}};

cv::Mat xv_dev_wrapper::toCvMatRGBD(const DepthColorImage& rgbd)
{
    cv::Mat out;
    out = cv::Mat::zeros(rgbd.height, rgbd.width*2, CV_8UC3);
    auto w = rgbd.width;

    if (rgbd.height>0 && rgbd.width>0) {
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<std::uint8_t const*>(rgbd.data.get()+3);
        for (unsigned int i=0; i< rgbd.height*rgbd.width; i++) {
            const auto &d = *reinterpret_cast<float const*>(tmp_d + i*(3+sizeof(float)));
            if( d < 0.01 || d > 9.9 ) {
                out.at<cv::Vec3b>(i / w, i % rgbd.width) = 0;
            } else {
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/ w, i%rgbd.width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
            }
        }
        const auto tmp_rgb = reinterpret_cast<std::uint8_t const*>(rgbd.data.get());
        for (unsigned int i=0; i< rgbd.height*rgbd.width; i++) {
            const auto rgb = reinterpret_cast<std::uint8_t const*>(tmp_rgb + i*(3+sizeof(float)));
            out.at<cv::Vec3b>( i/ w, (i%rgbd.width) + rgbd.width) = cv::Vec3b(rgb[0], rgb[1], rgb[2]);
        }
    }
    return out;
}

sensor_msgs::msg::Image xv_dev_wrapper::toRosImage(const DepthColorImage& xvDepthColorImage, const std::string& frame_id)
{
    if (xvDepthColorImage.hostTimestamp < 0)
        this->printWarningMsg("XVSDK-ROS-WRAPPER toRosImage() Error: negative DepthColorImage host-timestamp");

    sensor_msgs::msg::Image rosImage;
    rosImage.header.stamp = get_stamp_from_sec(xvDepthColorImage.hostTimestamp > 0.1 ? xvDepthColorImage.hostTimestamp : 0.1);
    rosImage.header.frame_id = frame_id;
    rosImage.height = xvDepthColorImage.height;
    rosImage.width = xvDepthColorImage.width * 2;
    rosImage.encoding = sensor_msgs::image_encodings::RGB8; /// FIXME why need BGR to get RGB ?
    rosImage.is_bigendian = false;
    rosImage.step = rosImage.width * 3*sizeof(uint8_t); /// bytes for 1 line
    const int nbPx = rosImage.width * rosImage.height;
    std::vector<uint8_t> copy(3*nbPx);
    cv::Mat cvMat = toCvMatRGBD(xvDepthColorImage);
    // std::memcpy(&copy[0], cvMat.data, nbPx * 3*sizeof(uint8_t));
    // /// TODO use that instead when implemented
    // //std::memcpy(&copy[0], xvColorImage.toRgb().data.get(), nbPx * 3*sizeof(uint8_t));
    // rosImage.data = std::move(copy);
    rosImage.data = std::vector<uint8_t>(cvMat.data, cvMat.data + nbPx * 3*sizeof(uint8_t));

    return rosImage;
}

sensor_msgs::msg::Image xv_dev_wrapper::toRosImageRaw(const DepthColorImage& xvDepthColorImage, const std::string& frame_id)
{
    if (xvDepthColorImage.hostTimestamp < 0)
        this->printWarningMsg("XVSDK-ROS-WRAPPER toRosImageRaw() Error: negative DepthColorImage host-timestamp");

    sensor_msgs::msg::Image rosImage;
    rosImage.header.stamp = get_stamp_from_sec(xvDepthColorImage.hostTimestamp > 0.1 ? xvDepthColorImage.hostTimestamp : 0.1);
    rosImage.header.frame_id = frame_id;
    rosImage.height = xvDepthColorImage.height;
    rosImage.width = xvDepthColorImage.width;
    rosImage.encoding = sensor_msgs::image_encodings::RGB8; /// FIXME why need BGR to get RGB ?
    rosImage.is_bigendian = false;
    rosImage.step = rosImage.width * 7*sizeof(uint8_t); /// bytes for 1 line
    const int nbPx = rosImage.width * rosImage.height;
    rosImage.data = std::vector<uint8_t>(xvDepthColorImage.data.get(), xvDepthColorImage.data.get() + nbPx * 7*sizeof(uint8_t));

    return rosImage;
}

cv::Mat xv_dev_wrapper::toCvMatRGB(const ColorImage& xvColorImage)
{
  cv::Mat cvMat;
  switch(xvColorImage.codec){
     case ColorImage::Codec::YUYV:{
         cv::Mat img( xvColorImage.height, xvColorImage.width, CV_8UC2, (void*)xvColorImage.data.get() );
         cv::cvtColor( img, cvMat, cv::COLOR_YUV2RGB_YUYV );
         break;
     }
     case ColorImage::Codec::YUV420p:{
         cv::Mat img( static_cast<int>(1.5*xvColorImage.height), xvColorImage.width, CV_8UC1, (void*)xvColorImage.data.get() );
         cv::cvtColor( img, cvMat, cv::COLOR_YUV420p2RGB );
         break;
     }
     case ColorImage::Codec::JPEG:{
         cv::Mat img( xvColorImage.height, xvColorImage.width, CV_8UC3, (void*)xvColorImage.data.get() );
         img = cv::imdecode( img, cv::IMREAD_COLOR );
         cv::cvtColor( img, cvMat, cv::COLOR_BGR2RGB );
         break;
     }
     case ColorImage::Codec::NV12:{
         cv::Mat img = cv::Mat( xvColorImage.height * 3/2, xvColorImage.width, CV_8UC1, (void*)xvColorImage.data.get() );
#if (CV_VERSION_MAJOR >= 4)
         cv::cvtColor(img, cvMat, cv::COLOR_YUV2RGB_YV12);
#else
         cv::cvtColor(img, cvMat, CV_YUV2RGB_YV12);
#endif
         break;
     }
     default: {
         /// TODO support BITSTREAM type
         this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER Error: unsupported color image type");
     }
  }

  return cvMat;
}

void xv_dev_wrapper::toRosOrientationStamped(xv_ros2_msgs::msg::OrientationStamped& orientation, Orientation const& xvOrientation, const std::string& frame_id)
{
  if (xvOrientation.hostTimestamp < 0)
    this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosOrientationStamped() Error: negative Orientation host-timestamp");

  orientation.header.stamp = get_stamp_from_sec(xvOrientation.hostTimestamp > 0.1 ? xvOrientation.hostTimestamp : 0.1);
  orientation.header.frame_id = frame_id;

  for (int i = 0; i < 9; ++i)
    orientation.matrix[i] = xvOrientation.rotation()[i];

  auto quat = xvOrientation.quaternion(); /// [qx,qy,qz,qw]
  orientation.quaternion.x = quat[0];
  orientation.quaternion.y = quat[1];
  orientation.quaternion.z = quat[2];
  orientation.quaternion.w = quat[3];

  orientation.angular_velocity.x = xvOrientation.angularVelocity()[0];
  orientation.angular_velocity.y = xvOrientation.angularVelocity()[1];
  orientation.angular_velocity.z = xvOrientation.angularVelocity()[2];
}

void xv_dev_wrapper::toRosColorDepthData(xv_ros2_msgs::msg::ColorDepth& colorDephData, const std::string& frame_id)
{
    if (m_xvDepthImage.hostTimestamp < 0 || m_xvColorImage.hostTimestamp < 0)
    this->m_node->printErrorMsg("XVSDK-ROS-WRAPPER toRosColorDepthData() Error: negative host-timestamp");

    colorDephData.header.stamp = get_stamp_from_sec(m_xvDepthImage.hostTimestamp > 0.1 ? m_xvDepthImage.hostTimestamp : 0.1);
    colorDephData.header.frame_id = frame_id;

    colorDephData.height = m_xvColorImage.height;
    colorDephData.width = m_xvColorImage.width;

    if(m_xvColorImage.data)
    {
        cv::Mat cvMat = toCvMatRGB(m_xvColorImage);
        colorDephData.rgb = std::vector<unsigned char>(cvMat.data, cvMat.data + m_xvColorImage.height * m_xvColorImage.width * 3 * sizeof(uint8_t));    }

    if(m_xvDepthImage.data)
    {
        if (m_xvDepthImage.type == xv::DepthImage::Type::Depth_32) {
            const auto depth = reinterpret_cast<float const*>(m_xvDepthImage.data.get());
            for(std::size_t i = 0; i < m_xvDepthImage.height*m_xvDepthImage.width; i++)
            {
                colorDephData.depth.push_back(depth[i]);
            }
        } else if (m_xvDepthImage.type == xv::DepthImage::Type::Depth_16) {
            const auto depth = reinterpret_cast<short const*>(m_xvDepthImage.data.get());
            for(std::size_t i = 0; i < m_xvDepthImage.height*m_xvDepthImage.width; i++)
            {
                colorDephData.depth.push_back((float)depth[i]/1000);
            }
        }
    }
}

std::filebuf mapStream;
std::atomic_int localized_on_reference_percent(0);

void cslamSavedCallback(int status_of_saved_map, int map_quality)
{

    std::cout << " Save map (quality is " << map_quality << "/100) and switch to CSlam:";
    switch (status_of_saved_map)
    {
    case  2: std::cout << " Map well saved. " << std::endl; break;
    case -1: std::cout << " Map cannot be saved, an error occured when trying to save it." << std::endl; break;
    default: std::cout << " Unrecognized status of saved map " << std::endl; break;
    }
    mapStream.close();
}

void cslamSwitchedCallback(int map_quality)
{
    std::cout << " map (quality is " << map_quality << "/100) and switch to CSlam:";
    mapStream.close();
}

void cslamLocalizedCallback(float percent)
{
    static int k = 0;
    if (k++ % 100 == 0) {
        localized_on_reference_percent = static_cast<int>(percent * 100);
        std::cout << "localized: " << localized_on_reference_percent << "%" << std::endl;
    }
}

bool xv_dev_wrapper::saveCslamMap(std::string filename)
{
    if (mapStream.open(filename.c_str(), std::ios::binary | std::ios::out | std::ios::trunc) == nullptr) {
        this->printErrorMsg("could not create map file");
        return false;
    }
    return m_device->slam()->saveMapAndSwitchToCslam(mapStream, cslamSavedCallback, cslamLocalizedCallback);
}

bool xv_dev_wrapper::loadCslamMap(std::string filename)
{
    if (mapStream.open(filename.c_str(), std::ios::binary | std::ios::in) == nullptr) {
        this->printErrorMsg("could not load map file");
        return false;
    }
    return m_device->slam()->loadMapAndSwitchToCslam(mapStream, cslamSwitchedCallback, cslamLocalizedCallback);
}

xv_ros2_msgs::msg::Controller xv_dev_wrapper::toRosControllerData(const WirelessControllerData data, const std::string& frame_id)
{
    xv_ros2_msgs::msg::Controller controllerData;
    controllerData.header.stamp = get_stamp_from_sec(data.pose.hostTimestamp() > 0.1 ? data.pose.hostTimestamp() : 0.1);
    controllerData.header.frame_id = frame_id;

    controllerData.keytrigger = data.keyTrigger;
    controllerData.keyside = data.keySide;
    controllerData.rockerx = data.rocker_x;
    controllerData.rockery = data.rocker_y;
    controllerData.key = data.key;

    return controllerData;
}

bool xv_dev_wrapper::startController(std::string portAddress)
{
    if(!portAddress.empty())
    {
        if(m_device->wirelessController())
        {
            this->printInfoMsg("controller set port name: " + portAddress);
            m_device->wirelessController()->setSerialPointName(portAddress);
            this->printInfoMsg("wireless controller start");
            m_device->wirelessController()->start();

            // Check if map_optical_frame has been published already
            std::string map_frame = this->m_node->getFrameID("map_optical_frame");
            if (!this->m_node->isFramePublished(map_frame)) {
                geometry_msgs::msg::TransformStamped static_transform;
                static_transform.header.stamp = rclcpp::Clock().now();
                static_transform.header.frame_id = "world";
                static_transform.child_frame_id = map_frame;
                static_transform.transform.translation.x = 0.0;
                static_transform.transform.translation.y = 0.0;
                static_transform.transform.translation.z = 0.0;
                static_transform.transform.rotation.x = 0.0;
                static_transform.transform.rotation.y = 0.0;
                static_transform.transform.rotation.z = 0.0;
                static_transform.transform.rotation.w = 1.0;
                this->m_node->broadcasterStaticTfTransform(static_transform);
                this->m_node->setFramePublished(map_frame);
            }

            m_controllerCBID = m_device->wirelessController()->registerWirelessControllerDataCallback([this](const xv::WirelessControllerData& data)
            {
                if(data.type == xv::WirelessControllerDataType::LEFT)
                {
                    // this->printInfoMsg("publish left controller pose data");
                    geometry_msgs::msg::PoseStamped poseSteamped = to_ros_poseEdgeStamped(data.pose, this->m_node->getFrameID("map_optical_frame"));
                    this->m_node->publishLeftControllerPose(m_sn,poseSteamped);

                    // Publish TF transform for left controller
                    geometry_msgs::msg::TransformStamped transform = toRosTransformStamped(data.pose,
                                                           this->m_node->getFrameID("map_optical_frame"),
                                                           "left_controller_frame");
                    // Ensure child_frame_id is set
                    if (transform.child_frame_id.empty()) {
                        transform.child_frame_id = "left_controller_frame";
                    }
                    this->m_node->broadcasterTfTransform(transform);

                    // this->printInfoMsg("publish left controller key data");
                    xv_ros2_msgs::msg::Controller controllerData = toRosControllerData(data, this->m_node->getFrameID("map_optical_frame"));
                    this->m_node->publishLeftControllerData(m_sn, controllerData);
                }
                else if(data.type == xv::WirelessControllerDataType::RIGHT)
                {
                    // this->printInfoMsg("publish right controller pose data");
                    geometry_msgs::msg::PoseStamped poseSteamped = to_ros_poseEdgeStamped(data.pose, this->m_node->getFrameID("map_optical_frame"));
                    this->m_node->publishRightControllerPose(m_sn, poseSteamped);

                    // Publish TF transform for right controller
                    geometry_msgs::msg::TransformStamped transform = toRosTransformStamped(data.pose,
                                                           this->m_node->getFrameID("map_optical_frame"),
                                                           "right_controller_frame");
                    // Ensure child_frame_id is set
                    if (transform.child_frame_id.empty()) {
                        transform.child_frame_id = "right_controller_frame";
                    }
                    this->m_node->broadcasterTfTransform(transform);

                    // this->printInfoMsg("publish right controller key data");
                    xv_ros2_msgs::msg::Controller controllerData = toRosControllerData(data, this->m_node->getFrameID("map_optical_frame"));
                    this->m_node->publishRightControllerData(m_sn, controllerData);
                }
            });
            return true;
        }
        else
        {
            this->printInfoMsg("wireless controller feature is invalid");
            return false;
        }
    }
    return false;
}

bool xv_dev_wrapper::stopController()
{
    m_device->wirelessController()->unregisterWirelessControllerDataCallback(m_controllerCBID);
    m_device->wirelessController()->stop();
    return true;
}

// Add these helper methods for consistent logging
void xv_dev_wrapper::printInfoMsg(const std::string& msgString) const
{
    m_node->printInfoMsg(msgString);
}

void xv_dev_wrapper::printWarningMsg(const std::string& msgString) const
{
    m_node->printWarningMsg(msgString);
}

void xv_dev_wrapper::printErrorMsg(const std::string& msgString) const
{
    m_node->printErrorMsg(msgString);
}
