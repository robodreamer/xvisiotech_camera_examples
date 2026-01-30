#ifndef XVISIO_CORE_DEVICE_H
#define XVISIO_CORE_DEVICE_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <xv-sdk.h>

namespace xvisio_core {

// Simple structs for pose and IMU data (ROS-free)
struct Pose {
  float position[3];
  float quaternion[4]; // w, x, y, z
  double host_timestamp_s;
  int64_t edge_timestamp_us;
  double confidence;
};

struct ImuSample {
  float accel[3];
  float gyro[3];
  double host_timestamp_s;
  int64_t edge_timestamp_us;
};

struct ControllerData {
  int type; // 0=left, 1=right
  float position[3];
  float quaternion[4]; // w, x, y, z
  double host_timestamp_s;
  uint8_t key_trigger;
  uint8_t key_side;
  int16_t rocker_x;
  int16_t rocker_y;
  uint8_t key;
};

struct DeviceInfo {
  std::string serial_number;
  std::string model;
  // Add more fields as needed
};

// ROS-free wrapper around xv::Device
class Device {
public:
  Device(std::shared_ptr<xv::Device> device, const std::string &serial_number);
  ~Device();

  // Device info
  std::string serial_number() const { return m_serial_number; }

  // SLAM control
  bool start_slam();
  bool stop_slam();
  bool slam_running() const { return m_slam_running; }

  // Pose queries
  bool get_pose(Pose &pose, double prediction_s = 0.0);
  bool get_pose_at(Pose &pose, double host_timestamp_s);

  // IMU control
  bool start_imu();
  bool stop_imu();
  bool imu_running() const { return m_imu_running; }

  // IMU queries (returns latest cached sample from callback)
  bool get_imu(ImuSample &imu);

  // Wireless controller (Seer)
  bool start_controller(const std::string &port);
  bool stop_controller();
  bool controller_running() const { return m_controller_running; }
  bool get_controller_data(ControllerData &left, ControllerData &right,
                           bool &left_available, bool &right_available);

private:
  void imu_callback(const xv::Imu &xv_imu);
  void fisheye_callback(const xv::FisheyeImages &images);
  void slam_callback(const xv::Pose &pose);
  void controller_callback(const xv::WirelessControllerData &data);

  std::shared_ptr<xv::Device> m_device;
  std::string m_serial_number;
  bool m_slam_running;
  bool m_imu_running;

  // IMU callback caching
  std::mutex m_imu_mutex;
  ImuSample m_latest_imu;
  bool m_imu_available;

  // Fisheye callback ID (for cleanup)
  int m_fisheye_callback_id;
  // SLAM callback ID (for cleanup)
  int m_slam_callback_id;

  // Wireless controller
  bool m_controller_running;
  int m_controller_callback_id;
  std::mutex m_controller_mutex;
  ControllerData m_controller_left;
  ControllerData m_controller_right;
  bool m_controller_left_available;
  bool m_controller_right_available;
};

// Device discovery
std::vector<DeviceInfo>
discover_devices(double timeout_s = 2.0,
                 xv::DeviceSupport device_support = xv::DeviceSupport::ONLYUSB);
std::shared_ptr<Device>
open_device(const std::string &serial_number = "", double timeout_s = 2.0,
            xv::DeviceSupport device_support = xv::DeviceSupport::ONLYUSB);

} // namespace xvisio_core

#endif // XVISIO_CORE_DEVICE_H
