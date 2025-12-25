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

    // SLAM requires fisheye cameras to be running for visual tracking
    // Start fisheye cameras if available (similar to ROS2 driver)
    if (m_device->fisheyeCameras()) {
        m_device->fisheyeCameras()->start();
    }

    m_slam_running = m_device->slam()->start();
    return m_slam_running;
}

bool Device::stop_slam() {
    if (!m_device || !m_device->slam()) {
        return false;
    }
    if (m_slam_running) {
        m_device->slam()->stop();
        m_slam_running = false;

        // Stop fisheye cameras when stopping SLAM
        if (m_device->fisheyeCameras()) {
            m_device->fisheyeCameras()->stop();
        }
    }
    return true;
}

bool Device::get_pose(Pose& pose, double prediction_s) {
    if (!m_device || !m_device->slam() || !m_slam_running) {
        return false;
    }

    xv::Pose xv_pose;
    bool ok = m_device->slam()->getPose(xv_pose, prediction_s);
    if (!ok) {
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

