#include "xvisio_core/device.h"
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace xvisio_core;

NB_MODULE(_xvisio_impl, m) {
  m.doc() = "Xvisio device bindings";

  // DeviceInfo
  nb::class_<DeviceInfo>(m, "DeviceInfo")
      .def_ro("serial_number", &DeviceInfo::serial_number)
      .def_ro("model", &DeviceInfo::model);

  // Pose
  nb::class_<Pose>(m, "Pose")
      .def_prop_ro("position",
                   [](const Pose &p) {
                     return nb::make_tuple(p.position[0], p.position[1],
                                           p.position[2]);
                   })
      .def_prop_ro("quaternion",
                   [](const Pose &p) {
                     // w, x, y, z
                     return nb::make_tuple(p.quaternion[0], p.quaternion[1],
                                           p.quaternion[2], p.quaternion[3]);
                   })
      .def_ro("host_timestamp_s", &Pose::host_timestamp_s)
      .def_ro("edge_timestamp_us", &Pose::edge_timestamp_us)
      .def_ro("confidence", &Pose::confidence);

  // ImuSample
  nb::class_<ImuSample>(m, "ImuSample")
      .def_prop_ro("accel",
                   [](const ImuSample &imu) {
                     return nb::make_tuple(imu.accel[0], imu.accel[1],
                                           imu.accel[2]);
                   })
      .def_prop_ro("gyro",
                   [](const ImuSample &imu) {
                     return nb::make_tuple(imu.gyro[0], imu.gyro[1],
                                           imu.gyro[2]);
                   })
      .def_ro("host_timestamp_s", &ImuSample::host_timestamp_s)
      .def_ro("edge_timestamp_us", &ImuSample::edge_timestamp_us);

  // Device
  nb::class_<Device>(m, "Device")
      .def("serial_number", &Device::serial_number)
      .def("start_slam", &Device::start_slam)
      .def("stop_slam", &Device::stop_slam)
      .def("slam_running", &Device::slam_running)
      .def(
          "get_pose",
          [](Device &dev, double prediction_s) {
            Pose pose;
            bool ok = dev.get_pose(pose, prediction_s);
            if (!ok) {
              throw std::runtime_error("Failed to get pose");
            }
            return pose;
          },
          nb::arg("prediction_s") = 0.0)
      .def("get_pose_at",
           [](Device &dev, double host_timestamp_s) {
             Pose pose;
             bool ok = dev.get_pose_at(pose, host_timestamp_s);
             if (!ok) {
               throw std::runtime_error("Failed to get pose at timestamp");
             }
             return pose;
           })
      .def("start_imu", &Device::start_imu)
      .def("stop_imu", &Device::stop_imu)
      .def("imu_running", &Device::imu_running)
      .def("get_imu", [](Device &dev) {
        ImuSample imu;
        bool ok = dev.get_imu(imu);
        if (!ok) {
          throw std::runtime_error("Failed to get IMU");
        }
        return imu;
      });

  // Module-level functions
  m.def("discover_devices", &discover_devices,
        "Discover available Xvisio devices");
  m.def("open_device", &open_device, nb::arg("serial_number") = "",
        "Open a device by serial number (empty string for first device)");
}
