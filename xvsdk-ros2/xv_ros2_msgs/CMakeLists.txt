cmake_minimum_required(VERSION 3.8)
project(xv_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OrientationStamped.msg"
  "msg/ColorDepth.msg"
  "srv/GetOrientation.srv"
  "srv/GetOrientationAt.srv"
  "srv/GetPose.srv"
  "srv/GetPoseAt.srv"
  "srv/LoadMapAndSwitchCslam.srv"
  "srv/SaveMapAndSwitchCslam.srv"
  "srv/ControllerStart.srv"
  "srv/ControllerStop.srv"
  "msg/Controller.msg"
  "msg/EventData.msg"
  "msg/ButtonMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
