cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

# 查找并包含 Gazebo
find_package(gazebo REQUIRED)

# 查找并包含 ROS
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找并包含 gazebo_ros
find_package(gazebo_ros REQUIRED)


# 设置编译选项
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件目录
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# 添加插件目标
add_library(parabolic_motion SHARED
  src/parabolic_motion.cpp
)

add_library(mini_motion SHARED
  src/mini_motion.cpp
)

# 链接依赖库
target_link_libraries(parabolic_motion
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  
)

target_link_libraries(mini_motion
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  
)

# 设置插件输出路径
set_target_properties(parabolic_motion PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugins
)
set_target_properties(mini_motion PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugins
)

# 将插件安装到 Gazebo 插件路径中
install(TARGETS parabolic_motion
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)
install(TARGETS mini_motion
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)