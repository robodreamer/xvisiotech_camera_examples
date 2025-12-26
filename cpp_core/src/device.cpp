#include "xvisio_core/device.h"
#include <xv-sdk.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <mutex>

namespace xvisio_core {

Device::Device(std::shared_ptr<xv::Device> device, const std::string& serial_number)
    : m_device(device)
    , m_serial_number(serial_number)
    , m_slam_running(false)
    , m_imu_running(false)
    , m_imu_available(false)
    , m_fisheye_callback_id(-1)
    , m_slam_callback_id(-1)
{
    // Initialize latest IMU sample
    m_latest_imu.accel[0] = 0.0f;
    m_latest_imu.accel[1] = 0.0f;
    m_latest_imu.accel[2] = 0.0f;
    m_latest_imu.gyro[0] = 0.0f;
    m_latest_imu.gyro[1] = 0.0f;
    m_latest_imu.gyro[2] = 0.0f;
    m_latest_imu.host_timestamp_s = 0.0;
    m_latest_imu.edge_timestamp_us = 0;
}

Device::~Device() {
    if (m_slam_running) {
        stop_slam();
    }
    if (m_imu_running) {
        stop_imu();
    }
}

bool Device::start_slam() {
    if (!m_device || !m_device->slam()) {
        return false;
    }

    // Start IMU first if available (SLAM benefits from IMU data during initialization)
    // This matches ROS2 driver initialization order: initImu() -> initFisheyeCameras() -> initSlam()
    if (m_device->imuSensor() && !m_imu_running) {
        start_imu();
    }

    // SLAM requires fisheye cameras to be running for visual tracking
    // Start fisheye cameras if available (similar to ROS2 driver)
    // CRITICAL: Must register a callback to consume fisheye frames, otherwise SLAM won't work
    if (m_device->fisheyeCameras()) {
        // Register callback to consume fisheye frames (required for SLAM to process visual data)
        auto fe_callback = [this](const xv::FisheyeImages& images) {
            this->fisheye_callback(images);
        };
        m_fisheye_callback_id = m_device->fisheyeCameras()->registerCallback(fe_callback);
        if (m_fisheye_callback_id < 0) {
            return false;  // Callback registration failed
        }
        if (!m_device->fisheyeCameras()->start()) {
            m_device->fisheyeCameras()->unregisterCallback(m_fisheye_callback_id);
            m_fisheye_callback_id = -1;
            return false;  // Failed to start fisheye cameras
        }
    }

    // Register SLAM callback BEFORE starting (required for SLAM to process poses)
    // Even though we poll with getPose(), registering a callback ensures SLAM is active
    auto slam_cb = [this](const xv::Pose& pose) {
        this->slam_callback(pose);
    };
    m_slam_callback_id = m_device->slam()->registerCallback(slam_cb);
    if (m_slam_callback_id < 0) {
        // Clean up fisheye on failure
        if (m_device->fisheyeCameras() && m_fisheye_callback_id >= 0) {
            m_device->fisheyeCameras()->stop();
            m_device->fisheyeCameras()->unregisterCallback(m_fisheye_callback_id);
            m_fisheye_callback_id = -1;
        }
        return false;  // Callback registration failed
    }

    // Start SLAM - this may take a moment to initialize
    m_slam_running = m_device->slam()->start();

    if (!m_slam_running) {
        // Clean up on failure
        if (m_slam_callback_id >= 0) {
            m_device->slam()->unregisterCallback(m_slam_callback_id);
            m_slam_callback_id = -1;
        }
        if (m_device->fisheyeCameras() && m_fisheye_callback_id >= 0) {
            m_device->fisheyeCameras()->stop();
            m_device->fisheyeCameras()->unregisterCallback(m_fisheye_callback_id);
            m_fisheye_callback_id = -1;
        }
    }

    return m_slam_running;
}

bool Device::stop_slam() {
    if (!m_device || !m_device->slam()) {
        return false;
    }
    if (m_slam_running) {
        m_device->slam()->stop();
        m_slam_running = false;

        // Unregister SLAM callback
        if (m_slam_callback_id >= 0) {
            m_device->slam()->unregisterCallback(m_slam_callback_id);
            m_slam_callback_id = -1;
        }

        // Stop fisheye cameras when stopping SLAM
        if (m_device->fisheyeCameras()) {
            m_device->fisheyeCameras()->stop();
            // Unregister callback
            if (m_fisheye_callback_id >= 0) {
                m_device->fisheyeCameras()->unregisterCallback(m_fisheye_callback_id);
                m_fisheye_callback_id = -1;
            }
        }
    }
    return true;
}

bool Device::get_pose(Pose& pose, double prediction_s) {
    if (!m_device || !m_device->slam()) {
        return false;
    }

    if (!m_slam_running) {
        return false;
    }

    xv::Pose xv_pose;
    bool ok = m_device->slam()->getPose(xv_pose, prediction_s);
    if (!ok) {
        // SLAM might not have initialized yet - this is normal during startup
        // The callback will be called once SLAM starts tracking
        return false;
    }

    // Convert xv::Pose to our Pose struct
    // ROS wrapper uses xvPose.x()/y()/z() and quaternion() as [qx,qy,qz,qw].
    pose.position[0] = xv_pose.x();
    pose.position[1] = xv_pose.y();
    pose.position[2] = xv_pose.z();

    const auto quat_xyzw = xv_pose.quaternion();  // [qx, qy, qz, qw]
    // Store as [w, x, y, z] for Python API.
    pose.quaternion[0] = quat_xyzw[3];
    pose.quaternion[1] = quat_xyzw[0];
    pose.quaternion[2] = quat_xyzw[1];
    pose.quaternion[3] = quat_xyzw[2];

    pose.host_timestamp_s = xv_pose.hostTimestamp();
    pose.edge_timestamp_us = xv_pose.edgeTimestampUs();
    pose.confidence = xv_pose.confidence();

    return true;
}

bool Device::get_pose_at(Pose& pose, double host_timestamp_s) {
    if (!m_device || !m_device->slam() || !m_slam_running) {
        return false;
    }

    xv::Pose xv_pose;
    bool ok = m_device->slam()->getPoseAt(xv_pose, host_timestamp_s);
    if (!ok) {
        return false;
    }

    // Convert xv::Pose to our Pose struct
    pose.position[0] = xv_pose.x();
    pose.position[1] = xv_pose.y();
    pose.position[2] = xv_pose.z();

    const auto quat_xyzw = xv_pose.quaternion();  // [qx, qy, qz, qw]
    pose.quaternion[0] = quat_xyzw[3];
    pose.quaternion[1] = quat_xyzw[0];
    pose.quaternion[2] = quat_xyzw[1];
    pose.quaternion[3] = quat_xyzw[2];

    pose.host_timestamp_s = xv_pose.hostTimestamp();
    pose.edge_timestamp_us = xv_pose.edgeTimestampUs();
    pose.confidence = xv_pose.confidence();

    return true;
}

bool Device::start_imu() {
    if (!m_device || !m_device->imuSensor()) {
        return false;
    }

    // Register callback to cache latest IMU sample
    auto imu_callback_fun = [this](const xv::Imu& xv_imu) {
        this->imu_callback(xv_imu);
    };
    m_device->imuSensor()->registerCallback(imu_callback_fun);

    m_imu_running = m_device->imuSensor()->start();
    return m_imu_running;
}

bool Device::stop_imu() {
    if (!m_device || !m_device->imuSensor()) {
        return false;
    }
    if (m_imu_running) {
        m_device->imuSensor()->stop();
        m_imu_running = false;
    }
    return true;
}

void Device::imu_callback(const xv::Imu& xv_imu) {
    std::lock_guard<std::mutex> lock(m_imu_mutex);

    // Convert xv::Imu to our ImuSample struct.
    // ROS wrapper uses xvImu.gyro[0..2], xvImu.accel[0..2], and xvImu.hostTimestamp fields.
    m_latest_imu.accel[0] = xv_imu.accel[0];
    m_latest_imu.accel[1] = xv_imu.accel[1];
    m_latest_imu.accel[2] = xv_imu.accel[2];

    m_latest_imu.gyro[0] = xv_imu.gyro[0];
    m_latest_imu.gyro[1] = xv_imu.gyro[1];
    m_latest_imu.gyro[2] = xv_imu.gyro[2];

    m_latest_imu.host_timestamp_s = xv_imu.hostTimestamp;
    // Not all SDK variants expose an edge timestamp for IMU; keep 0 in v1.
    m_latest_imu.edge_timestamp_us = 0;

    m_imu_available = true;
}

bool Device::get_imu(ImuSample& imu) {
    if (!m_device || !m_device->imuSensor() || !m_imu_running) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_imu_mutex);
    if (!m_imu_available) {
        return false;
    }

    imu = m_latest_imu;
    return true;
}

void Device::fisheye_callback(const xv::FisheyeImages& images) {
    // Minimal callback - just consume the frames to keep the stream active
    // SLAM needs fisheye frames to be consumed for visual tracking to work
    // We don't need to do anything with the data, just registering the callback
    // ensures the SDK processes the frames and feeds them to SLAM
    (void)images;  // Suppress unused parameter warning
}

void Device::slam_callback(const xv::Pose& pose) {
    // Minimal callback - SLAM needs a callback registered to be active
    // Even though we poll with getPose(), registering a callback ensures
    // SLAM processes poses internally
    (void)pose;  // Suppress unused parameter warning
}

std::vector<DeviceInfo> discover_devices() {
    std::vector<DeviceInfo> devices;

    // Set log level to err to suppress info/warn messages (including stereo frame warnings)
    // Users can still see errors and critical messages
    xv::setLogLevel(xv::LogLevel::err);
    std::map<std::string, std::shared_ptr<xv::Device>> device_map;

    // Try to read config file if it exists
    std::string json = "";
    std::string jsonPath = "/etc/xvisio/config.json";
    std::ifstream ifs(jsonPath);
    if (ifs.is_open()) {
        std::stringstream fbuf;
        fbuf << ifs.rdbuf();
        json = fbuf.str();
        ifs.close();
        device_map = xv::getDevices(0.0, json);
    } else {
        device_map = xv::getDevices(0.0);
    }

    for (const auto& pair : device_map) {
        DeviceInfo info;
        info.serial_number = pair.first;
        // Could extract model from device if available
        info.model = "XR-50";  // Default assumption
        devices.push_back(info);
    }

    return devices;
}

std::shared_ptr<Device> open_device(const std::string& serial_number) {
    // Set log level to err to suppress info/warn messages (including stereo frame warnings)
    // Users can still see errors and critical messages
    xv::setLogLevel(xv::LogLevel::err);
    std::map<std::string, std::shared_ptr<xv::Device>> device_map;

    // Try to read config file if it exists
    std::string json = "";
    std::string jsonPath = "/etc/xvisio/config.json";
    std::ifstream ifs(jsonPath);
    if (ifs.is_open()) {
        std::stringstream fbuf;
        fbuf << ifs.rdbuf();
        json = fbuf.str();
        ifs.close();
        device_map = xv::getDevices(0.0, json);
    } else {
        device_map = xv::getDevices(0.0);
    }

    if (device_map.empty()) {
        return nullptr;
    }

    // If serial_number is empty, use first device
    std::string sn = serial_number.empty() ? device_map.begin()->first : serial_number;

    auto it = device_map.find(sn);
    if (it == device_map.end()) {
        return nullptr;
    }

    return std::make_shared<Device>(it->second, sn);
}

} // namespace xvisio_core

