#ifndef XVISIO_CORE_DEVICE_H
#define XVISIO_CORE_DEVICE_H

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <xv-sdk.h>

namespace xvisio_core {

// Simple structs for pose and IMU data (ROS-free)
struct Pose {
    float position[3];
    float quaternion[4];  // w, x, y, z
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

struct DeviceInfo {
    std::string serial_number;
    std::string model;
    // Add more fields as needed
};

// ROS-free wrapper around xv::Device
class Device {
public:
    Device(std::shared_ptr<xv::Device> device, const std::string& serial_number);
    ~Device();

    // Device info
    std::string serial_number() const { return m_serial_number; }

    // SLAM control
    bool start_slam();
    bool stop_slam();
    bool slam_running() const { return m_slam_running; }

    // Pose queries
    bool get_pose(Pose& pose, double prediction_s = 0.0);
    bool get_pose_at(Pose& pose, double host_timestamp_s);

    // IMU control
    bool start_imu();
    bool stop_imu();
    bool imu_running() const { return m_imu_running; }

    // IMU queries (returns latest cached sample from callback)
    bool get_imu(ImuSample& imu);

private:
    void imu_callback(const xv::Imu& xv_imu);
    void fisheye_callback(const xv::FisheyeImages& images);
    void slam_callback(const xv::Pose& pose);

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
};

// Device discovery
std::vector<DeviceInfo> discover_devices();
std::shared_ptr<Device> open_device(const std::string& serial_number = "");

} // namespace xvisio_core

#endif // XVISIO_CORE_DEVICE_H

