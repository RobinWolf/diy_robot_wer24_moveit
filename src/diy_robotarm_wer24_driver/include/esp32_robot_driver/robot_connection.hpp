#ifndef ROBOT_CONNECTION_HPP_
#define ROBOT_CONNECTION_HPP_

#include <iostream>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <array>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <functional>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "data_format.hpp"

class RobotConnection;

struct RobotConnectionArgs {
  RobotConnection * robot;
  std::string ipAddress;
  uint8_t port;
} connectionConfig;

Communication::PcToRobot_t request;
Communication::RobotToPc_t response;

pthread_t threadId_;

class RobotConnection {
private:
  uint8_t messageNumber_ = 0;
  int socketConnection_;

  int bytesSent = 0;

  bool newMessageRevceived_ = false;
  std::string newMessage_ = "";

public:
  // Arrays to store information from/ for ros2_control Interfaces (state and command):
  std::vector<double> hw_cmd_axisSetpoints;     // Axis Setpoints (command)
  std::vector<double> hw_states_axisPositions;  // Axis Position (state)


  //#####################################################################################
  // called in driver: "ESP32Hardware::on_activate" and "ESP32Hardware::on_deactivate"
  //#####################################################################################
  // Enable or disable the actuator power

  void toggleActuatorPower(bool enable = false) {
    request.enablePower = enable;
  };


  //#####################################################################################
  // called in driver: "ESP32Hardware::on_configure"
  //##################################################################################### 
  // Connect to the robot hardware on the given IP Address. 

  bool initialize(std::string ipAddress) {

    // Create a socket handle:
    socketConnection_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socketConnection_ == -1) { std::cerr << "Error creating socket" << std::endl; return false; }   

    // Setup the network connection parameters:
    struct sockaddr_in receiverAddr;
    receiverAddr.sin_family = AF_INET;
    receiverAddr.sin_port = htons(80);
    receiverAddr.sin_addr.s_addr = inet_addr(ipAddress.c_str());
    
    request.messageNumber = 0;

    // Try to connect to the robot:
    if (connect(socketConnection_, (struct sockaddr*) &receiverAddr, sizeof(receiverAddr)) == -1) {
      std::cerr << "Error connecting to the Robot" << std::endl;
      return false;
    }
    return true;
  };


  //#####################################################################################
  // called in driver: "ESP32Hardware::read"
  //#####################################################################################

  bool readData() {
    if (bytesSent == 0) return true;  // Return if no response is expected

    // Read data from the socket
    int bytesReceived = recv(socketConnection_, &response, sizeof(response), 0);
    
    // Update the axis position states from the received values:
    for (auto i = 0; i < hw_states_axisPositions.size(); i++) {
      hw_states_axisPositions[i] = (double) response.jointPositions[i] / 57295.7795131; //conversion millidegrees to rad (esp works in millidegrees, ros in rad)
    }
  }


  //#####################################################################################
  // called in driver: "ESP32Hardware::write"
  //#####################################################################################

  bool sendData() {
    // Copy the axis setpoints to the request message:
    for (auto i = 0; i < hw_cmd_axisSetpoints.size(); i++) {
      request.jointSetpoints[i] = (int32_t) (hw_cmd_axisSetpoints[i] * 57295.7795131);    //57295.7795131 = 1000 * 180/pi
    } 
    
    // Start send the data to the robot (increasing the message number for checking purposes):
    request.messageNumber++;
    bytesSent = send(socketConnection_, &request, sizeof(request), 0);

    // Raise an error if no bytes have been sent:
    if (bytesSent == -1) {
      std::cerr << "Error while sending Message to the ESP via TCP/IP";
      return false;
    }
    return true;
  }

};

#endif