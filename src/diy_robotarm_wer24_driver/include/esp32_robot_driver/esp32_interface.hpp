#ifndef ESP32_ROBOT_DRIVER__ESP32_INTERFACE_HPP_
#define ESP32_ROBOT_DRIVER__ESP32_INTERFACE_HPP_

//Header File for the Controller talking to our ESP32 Hardware --> construct following public methods for overriding the inheritance ControllerInterface definition

// Tutorial: https://control.ros.org/humble/doc/ros2_controllers/doc/writing_new_controller.html

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"


// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


// define the hardware inteface (type system: read and write capabilities) which the controller can access
namespace esp32_robot_driver{

  class ESP32Hardware:  public hardware_interface::SystemInterface {
  public:
    virtual ~ESP32Hardware();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override; //declares a member function named on_init. It is declared as an override of a function in the base class

    hardware_interface::CallbackReturn on_configure(const   rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  };
}

#endif