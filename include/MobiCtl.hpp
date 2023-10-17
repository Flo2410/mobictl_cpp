#ifndef MOBICTL_SENSORS_H_
#define MOBICTL_SENSORS_H_

#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "libserial/SerialPort.h"
#include "min.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "string"
#include "vector"

using LibSerial::SerialPort;

class MobiCtl : public rclcpp::Node {
 public:
  static std::shared_ptr<MobiCtl> mobictl() {
    static auto instance = std::make_shared<MobiCtl>();
    return instance;
  }

  // --------------------------------------------------
  // Begin generated code from protocol.json
  // --------------------------------------------------

  // commands
  enum class COMMANDS {
    IMU = 0x20,
    ULTRASONIC_SENSOR = 0x21,
    ENCODER = 0x22,
    BRIGHTNESS = 0x23,
    TEMPERATURE = 0x24,
    BAT_VOLTAGE = 0x25,
    USER_BUTTON = 0x26,
    LED_STRIP = 0x27,
    MOTOR_CONTROL = 0x28,
    POZYX_POWER = 0x29,
    POZYX = 0x2a,
    POZYX_CONFIG = 0x2b,
    DISABLE_ALL_INTERVALS = 0x3d,
    FIRMWARE_INFO = 0x3e,
  };

  // data
  enum class DATA {
    COMMAND_STAUTS = 0x0,
    IMU = 0x1,
    ULTRASONIC_SENSOR = 0x2,
    ENCODER = 0x3,
    BRIGHTNESS = 0x4,
    TEMPERATURE = 0x5,
    BAT_VOLTAGE = 0x6,
    USER_BUTTON = 0x7,
    POZYX = 0x8,
    POZYX_INFO = 0x9,
    POZYX_POWER_STATE = 0xa,
    FIRMWARE_INFO = 0x3f,
  };

  enum class IMU_SUB_DEVICES {
    ACCELEROMETER = 0x1,
    MAGNETOMETER = 0x2,
    GYROSCOPE = 0x4,
    EULER = 0x8,
    LINEAR_ACCEL = 0x10,
    GRAVITY = 0x20,
    QUATERNION = 0x40,
  };

  enum class ULTRASONIC_SENSOR_SUB_DEVICES {
    US_1 = 0x1,
    US_2 = 0x2,
    US_3 = 0x4,
    US_4 = 0x8,
    US_5 = 0x10,
    US_6 = 0x20,
  };

  enum class ENCODER_SUB_DEVICES {
    ENCODER_1 = 0x1,
    ENCODER_2 = 0x2,
    ENCODER_3 = 0x4,
    ENCODER_4 = 0x8,
  };

  enum class ANIMATION_PRESET {
    DRIVING_LIGHTS = 0x0,
    BEACON = 0x1,
    BLINK = 0x2,
    ON = 0x3,
  };

  enum class POZYX_SUB_DEVICES {
    POSITION = 0x1,
    EULER = 0x2,
    QUATERNION = 0x4,
  };

  enum class STATUS_CODE {
    OK = 0x0,
    ERROR = 0x1,
    INVALID_PARAMETER = 0x2,
    UNKOWN_COMMAND = 0x3,
    BATTERY_WARNING = 0x4,
  };

  // --------------------------------------------------
  // END generated code
  // --------------------------------------------------

  struct COLOR_RGBW {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
  };

  struct min_context min_ctx;
  std::vector<uint8_t> tx_queue;
  SerialPort serial_port;

  MobiCtl();
  void handle_min_frame(uint8_t min_id, uint8_t const* min_payload, uint8_t len_payload);
  void shutdown(void);
  void setup(void);

 private:
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ultra_1;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ultra_2;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ultra_3;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ultra_4;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ultra_5;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ultra_6;
  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr pub_illuminance;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_euler;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;

  rclcpp::TimerBase::SharedPtr timer_;
  bool is_serial_connected = false;

  sensor_msgs::msg::Imu last_imu_msg;

  std::vector<uint8_t> extract_subdevices_from_byte(uint8_t byte);
  std::string get_first_serial_port_name(void);

  void loop(void);

  void joy_callback(const sensor_msgs::msg::Joy& msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);

  void send_light_preset(void);
};

#endif
