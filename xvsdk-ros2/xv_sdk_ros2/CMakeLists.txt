cmake_minimum_required(VERSION 3.8)
project(xv_sdk_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(xv_ros2_msgs REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## Find system libraries
# find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

# find xvisio sdk
set(XVSDK_INCLUDE_DIRS "/usr/include/xvsdk")
set(XVSDK_LIBRARIES "/usr/lib/libxvsdk.so")
if (XVSDK_INCLUDE_DIRS AND XVSDK_LIBRARIES)
  set(XVSDK_FOUND TRUE)
else()
  set(XVSDK_FOUND FALSE)
endif()
if (XVSDK_FOUND)
  message(STATUS " - xvsdk includes: ${XVSDK_INCLUDE_DIRS}")
  message(STATUS " - xvsdk libraries: ${XVSDK_LIBRARIES}")
else()
  message(FATAL_ERROR "xvsdk not provided")
endif()

include_directories(
  include
  # Set manually because Eigen sets a non standard INCLUDE DIR
  #${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  #${Boost_INCLUDE_DIRS}
  #/usr/include/xvsdk
  ${XVSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include2/
)

add_compile_options(-Wno-pedantic)

add_executable(xv_cameras src/main.cpp src/xv_ros2_node.cpp src/xv_dev_wrapper.cpp)
target_link_libraries(xv_cameras ${XVSDK_LIBRARIES} pthread ${OpenCV_LIBS})
target_include_directories(xv_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(xv_cameras PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(xv_cameras
  rclcpp
  std_msgs
  sensor_msgs
  xv_ros2_msgs
  std_srvs
  builtin_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  image_transport)

install(TARGETS xv_cameras
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
