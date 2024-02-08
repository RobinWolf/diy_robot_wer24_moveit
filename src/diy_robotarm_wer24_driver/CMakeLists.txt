cmake_minimum_required(VERSION 3.8)
project(diy_robotarm_wer24_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(diy_robotarm_wer24_description REQUIRED)
find_package(diy_soft_gripper_description REQUIRED)
find_package(diy_robot_full_cell_description REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control)
find_package(ros2_controllers)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  diy_robotarm_wer24_description
  diy_soft_gripper_description
  diy_robot_full_cell_description
  xacro
  ros2_control
  ros2_controllers
)

#build and compile hardware interface
add_library(esp32_interface_plugin
  SHARED
  src/esp32_interface.cpp
)
target_link_libraries(
  esp32_interface_plugin
)
target_include_directories(
  esp32_interface_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  esp32_interface_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
#export self written hardware interface plugin to libary
pluginlib_export_plugin_description_file(hardware_interface esp32_interface_plugin.xml)


# INSTALL
install(
  TARGETS esp32_interface_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  esp32_interface_plugin
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)



ament_package()
