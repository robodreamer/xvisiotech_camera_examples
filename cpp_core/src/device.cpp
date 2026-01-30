#include "xvisio_core/device.h"
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <xv-sdk.h>

namespace xvisio_core {

Device::Device(std::shared_ptr<xv::Device> device,
               const std::string &serial_number)
    : m_device(device), m_serial_number(serial_number), m_slam_running(false),
      m_imu_running(false), m_imu_available(false), m_fisheye_callback_id(-1),
      m_slam_callback_id(-1), m_controller_running(false),
      m_controller_callback_id(-1), m_controller_left_available(false),
      m_controller_right_available(false) {
  // Initialize latest IMU sample
  m_latest_imu.accel[0] = 0.0f;
  m_latest_imu.accel[1] = 0.0f;
  m_latest_imu.accel[2] = 0.0f;
  m_latest_imu.gyro[0] = 0.0f;
  m_latest_imu.gyro[1] = 0.0f;
  m_latest_imu.gyro[2] = 0.0f;
  m_latest_imu.host_timestamp_s = 0.0;
  m_latest_imu.edge_timestamp_us = 0;

  // Initialize controller data
  m_controller_left.type = 0;
  m_controller_right.type = 1;
  for (int i = 0; i < 3; ++i) {
    m_controller_left.position[i] = 0.0f;
    m_controller_right.position[i] = 0.0f;
  }
  for (int i = 0; i < 4; ++i) {
    m_controller_left.quaternion[i] = (i == 0) ? 1.0f : 0.0f;
    m_controller_right.quaternion[i] = (i == 0) ? 1.0f : 0.0f;
  }
  m_controller_left.host_timestamp_s = 0.0;
  m_controller_right.host_timestamp_s = 0.0;
  m_controller_left.key_trigger = 0;
  m_controller_left.key_side = 0;
  m_controller_left.rocker_x = 0;
  m_controller_left.rocker_y = 0;
  m_controller_left.key = 0;
  m_controller_right.key_trigger = 0;
  m_controller_right.key_side = 0;
  m_controller_right.rocker_x = 0;
  m_controller_right.rocker_y = 0;
  m_controller_right.key = 0;
}

Device::~Device() {
  if (m_slam_running) {
    stop_slam();
  }
  if (m_imu_running) {
    stop_imu();
  }
  if (m_controller_running) {
    stop_controller();
  }
}

bool Device::start_slam() {
  if (!m_device || !m_device->slam()) {
    return false;
  }

  // Start IMU first if available (SLAM benefits from IMU data during
  // initialization) This matches ROS2 driver initialization order: initImu() ->
  // initFisheyeCameras() -> initSlam()
  if (m_device->imuSensor() && !m_imu_running) {
    start_imu();
  }

  // SLAM requires fisheye cameras to be running for visual tracking
  // Start fisheye cameras if available (similar to ROS2 driver)
  // CRITICAL: Must register a callback to consume fisheye frames, otherwise
  // SLAM won't work
  if (m_device->fisheyeCameras()) {
    // Register callback to consume fisheye frames (required for SLAM to process
    // visual data)
    auto fe_callback = [this](const xv::FisheyeImages &images) {
      this->fisheye_callback(images);
    };
    m_fisheye_callback_id =
        m_device->fisheyeCameras()->registerCallback(fe_callback);
    if (m_fisheye_callback_id < 0) {
      return false; // Callback registration failed
    }
    if (!m_device->fisheyeCameras()->start()) {
      m_device->fisheyeCameras()->unregisterCallback(m_fisheye_callback_id);
      m_fisheye_callback_id = -1;
      return false; // Failed to start fisheye cameras
    }
  }

  // Register SLAM callback BEFORE starting (required for SLAM to process poses)
  // Even though we poll with getPose(), registering a callback ensures SLAM is
  // active
  auto slam_cb = [this](const xv::Pose &pose) { this->slam_callback(pose); };
  m_slam_callback_id = m_device->slam()->registerCallback(slam_cb);
  if (m_slam_callback_id < 0) {
    // Clean up fisheye on failure
    if (m_device->fisheyeCameras() && m_fisheye_callback_id >= 0) {
      m_device->fisheyeCameras()->stop();
      m_device->fisheyeCameras()->unregisterCallback(m_fisheye_callback_id);
      m_fisheye_callback_id = -1;
    }
    return false; // Callback registration failed
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

bool Device::get_pose(Pose &pose, double prediction_s) {
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

  const auto quat_xyzw = xv_pose.quaternion(); // [qx, qy, qz, qw]
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

bool Device::get_pose_at(Pose &pose, double host_timestamp_s) {
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

  const auto quat_xyzw = xv_pose.quaternion(); // [qx, qy, qz, qw]
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
  auto imu_callback_fun = [this](const xv::Imu &xv_imu) {
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

void Device::imu_callback(const xv::Imu &xv_imu) {
  std::lock_guard<std::mutex> lock(m_imu_mutex);

  // Convert xv::Imu to our ImuSample struct.
  // ROS wrapper uses xvImu.gyro[0..2], xvImu.accel[0..2], and
  // xvImu.hostTimestamp fields.
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

bool Device::get_imu(ImuSample &imu) {
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

void Device::fisheye_callback(const xv::FisheyeImages &images) {
  // Minimal callback - just consume the frames to keep the stream active
  // SLAM needs fisheye frames to be consumed for visual tracking to work
  // We don't need to do anything with the data, just registering the callback
  // ensures the SDK processes the frames and feeds them to SLAM
  (void)images; // Suppress unused parameter warning
}

void Device::slam_callback(const xv::Pose &pose) {
  // Minimal callback - SLAM needs a callback registered to be active
  // Even though we poll with getPose(), registering a callback ensures
  // SLAM processes poses internally
  (void)pose; // Suppress unused parameter warning
}

void Device::controller_callback(const xv::WirelessControllerData &data) {
  std::lock_guard<std::mutex> lock(m_controller_mutex);
  const xv::Pose &p = data.pose;
  const auto quat_xyzw = p.quaternion(); // [qx, qy, qz, qw]
  if (data.type == xv::WirelessControllerDataType::LEFT) {
    m_controller_left.type = 0;
    m_controller_left.position[0] = p.x();
    m_controller_left.position[1] = p.y();
    m_controller_left.position[2] = p.z();
    m_controller_left.quaternion[0] = quat_xyzw[3];
    m_controller_left.quaternion[1] = quat_xyzw[0];
    m_controller_left.quaternion[2] = quat_xyzw[1];
    m_controller_left.quaternion[3] = quat_xyzw[2];
    m_controller_left.host_timestamp_s = p.hostTimestamp();
    m_controller_left.key_trigger = data.keyTrigger;
    m_controller_left.key_side = data.keySide;
    m_controller_left.rocker_x = data.rocker_x;
    m_controller_left.rocker_y = data.rocker_y;
    m_controller_left.key = data.key;
    m_controller_left_available = true;
  } else if (data.type == xv::WirelessControllerDataType::RIGHT) {
    m_controller_right.type = 1;
    m_controller_right.position[0] = p.x();
    m_controller_right.position[1] = p.y();
    m_controller_right.position[2] = p.z();
    m_controller_right.quaternion[0] = quat_xyzw[3];
    m_controller_right.quaternion[1] = quat_xyzw[0];
    m_controller_right.quaternion[2] = quat_xyzw[1];
    m_controller_right.quaternion[3] = quat_xyzw[2];
    m_controller_right.host_timestamp_s = p.hostTimestamp();
    m_controller_right.key_trigger = data.keyTrigger;
    m_controller_right.key_side = data.keySide;
    m_controller_right.rocker_x = data.rocker_x;
    m_controller_right.rocker_y = data.rocker_y;
    m_controller_right.key = data.key;
    m_controller_right_available = true;
  }
}

bool Device::start_controller(const std::string &port) {
  if (!m_device || !m_device->wirelessController()) {
    return false;
  }
  if (m_controller_running) {
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(m_controller_mutex);
    m_controller_left_available = false;
    m_controller_right_available = false;
  }
  m_device->wirelessController()->setSerialPointName(port);
  auto cb = [this](const xv::WirelessControllerData &data) {
    this->controller_callback(data);
  };
  m_controller_callback_id =
      m_device->wirelessController()->registerWirelessControllerDataCallback(
          cb);
  if (m_controller_callback_id < 0) {
    return false;
  }
  m_device->wirelessController()->start();
  m_controller_running = true;
  return true;
}

bool Device::stop_controller() {
  if (!m_device || !m_device->wirelessController()) {
    return false;
  }
  if (!m_controller_running) {
    return true;
  }
  if (m_controller_callback_id >= 0) {
    m_device->wirelessController()->unregisterWirelessControllerDataCallback(
        m_controller_callback_id);
    m_controller_callback_id = -1;
  }
  m_device->wirelessController()->stop();
  m_controller_running = false;
  m_controller_left_available = false;
  m_controller_right_available = false;
  return true;
}

bool Device::get_controller_data(ControllerData &left, ControllerData &right,
                                 bool &left_available, bool &right_available) {
  if (!m_controller_running) {
    left_available = false;
    right_available = false;
    return false;
  }
  std::lock_guard<std::mutex> lock(m_controller_mutex);
  left_available = m_controller_left_available;
  right_available = m_controller_right_available;
  if (m_controller_left_available) {
    left = m_controller_left;
  }
  if (m_controller_right_available) {
    right = m_controller_right;
  }
  return m_controller_left_available || m_controller_right_available;
}

std::vector<DeviceInfo> discover_devices(double timeout_s,
                                         xv::DeviceSupport device_support) {
  std::vector<DeviceInfo> devices;

  // Set log level to err to suppress info/warn messages (including stereo frame
  // warnings) Users can still see errors and critical messages
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
  }

  // For wireless controller, always use getDevices(0.0) -
  // getDevicesUntilTimeout doesn't work for controller discovery
  if (device_support == xv::DeviceSupport::ONLY_WIRELESS_CONTROLLER) {
    device_map = xv::getDevices(0.0, json, nullptr, xv::SlamStartMode::Normal,
                                device_support);
  } else if (timeout_s > 0.0) {
    device_map = xv::getDevicesUntilTimeout(
        timeout_s, json, xv::SlamStartMode::Normal, device_support);
  } else {
    device_map = xv::getDevices(0.0, json, nullptr, xv::SlamStartMode::Normal,
                                device_support);
  }

  for (const auto &pair : device_map) {
    DeviceInfo info;
    info.serial_number = pair.first;
    // Could extract model from device if available
    if (device_support == xv::DeviceSupport::ONLY_WIRELESS_CONTROLLER) {
      info.model = "Seer Controller";
    } else {
      info.model = "XR-50";
    }
    devices.push_back(info);
  }

  return devices;
}

std::shared_ptr<Device> open_device(const std::string &serial_number,
                                    double timeout_s,
                                    xv::DeviceSupport device_support) {
  // Set log level to err to suppress info/warn messages (including stereo frame
  // warnings) Users can still see errors and critical messages
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
  }

  // For wireless controller, always use getDevices(0.0) -
  // getDevicesUntilTimeout doesn't work for controller discovery
  if (device_support == xv::DeviceSupport::ONLY_WIRELESS_CONTROLLER) {
    device_map = xv::getDevices(0.0, json, nullptr, xv::SlamStartMode::Normal,
                                device_support);
  } else if (timeout_s > 0.0) {
    device_map = xv::getDevicesUntilTimeout(
        timeout_s, json, xv::SlamStartMode::Normal, device_support);
  } else {
    device_map = xv::getDevices(0.0, json, nullptr, xv::SlamStartMode::Normal,
                                device_support);
  }

  if (device_map.empty()) {
    return nullptr;
  }

  // If serial_number is empty, use first device
  std::string sn =
      serial_number.empty() ? device_map.begin()->first : serial_number;

  auto it = device_map.find(sn);
  if (it == device_map.end()) {
    return nullptr;
  }

  return std::make_shared<Device>(it->second, sn);
}

} // namespace xvisio_core
