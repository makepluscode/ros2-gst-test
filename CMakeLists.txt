cmake_minimum_required(VERSION 3.5)
project(gstcam)

set(CMAKE_CXX_STANDARD 14)

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# find Gstreamer
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_search_module(GST REQUIRED gstreamer-1.0 gstapp-1.0)

add_executable(gstcam_pub src/main.cpp)

ament_target_dependencies(gstcam_pub
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(gstcam_pub
  ${GST_LIBRARIES}
  gstapp-1.0
)

# include headers
target_include_directories(gstcam_pub PRIVATE
    ${GST_INCLUDE_DIRS}
    ${GLIB_INCLUDE_DIRS}
)

install(TARGETS
  gstcam_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()