#ifndef DATA_FORMAT_HPP_
#define DATA_FORMAT_HPP_

#include <stdint.h>
#include <queue>

#ifdef __linux__ 
#elif ESP32
#endif


// Error Codes
enum class ErrorCode : uint8_t {
  noError = 0,
  errorA,
  errorB,
  errorC
};

namespace Communication {

  union PcToRobot_t {
    struct __attribute__((packed)) {
      uint8_t messageNumber;
      bool emergencyStop;
      uint8_t reserved[1];

      bool enablePower;             // enable the axis
      int32_t jointSetpoints[6];    // Joint setpoints in millidegrees
    };

    uint8_t receiveBuffer[64];
  };// request;


  union RobotToPc_t {
    struct __attribute__((packed)) {
      uint8_t messageNumber;
      enum ErrorCode errorCode;
      uint8_t reserved[1];

      bool active;                  // feedback if axis are enabled ore disabled
      int32_t jointPositions[6];    // current jount positions in millidegreed (calculated on ESP side from counted steps)

    };
    uint8_t sendBuffer[64];
  };// response;
}

#endif