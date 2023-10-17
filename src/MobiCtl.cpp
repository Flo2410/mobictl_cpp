#include "MobiCtl.hpp"

#include <unistd.h>

#include <functional>
#include <memory>
#include <string>

#include "PayloadBuilder.hpp"
#include "chrono"
#include "geometry_msgs/msg/quaternion.hpp"
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
#include "vector"

using LibSerial::BaudRate;
using LibSerial::DataBuffer;
using std::placeholders::_1;

MobiCtl::MobiCtl() : Node("mobictl") {
#ifdef MOBICTL_DEBUG_PRINTING
  (void)rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
#endif

  RCLCPP_INFO(this->get_logger(), "Starting MobiCtl");
  RCLCPP_DEBUG(this->get_logger(), "DEBUG ENABLED!\n");

  this->pub_ultra_1 = this->create_publisher<sensor_msgs::msg::Range>("mobictl/ultra_1", 10);
  this->pub_ultra_2 = this->create_publisher<sensor_msgs::msg::Range>("mobictl/ultra_2", 10);
  this->pub_ultra_3 = this->create_publisher<sensor_msgs::msg::Range>("mobictl/ultra_3", 10);
  this->pub_ultra_4 = this->create_publisher<sensor_msgs::msg::Range>("mobictl/ultra_4", 10);
  this->pub_ultra_5 = this->create_publisher<sensor_msgs::msg::Range>("mobictl/ultra_5", 10);
  this->pub_ultra_6 = this->create_publisher<sensor_msgs::msg::Range>("mobictl/ultra_6", 10);
  this->pub_illuminance = this->create_publisher<sensor_msgs::msg::Illuminance>("mobictl/illuminance", 10);
  this->pub_temperature = this->create_publisher<sensor_msgs::msg::Temperature>("mobictl/temperature", 10);
  this->pub_battery = this->create_publisher<sensor_msgs::msg::BatteryState>("mobictl/battery", 10);
  this->pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("mobictl/imu", 10);
  this->pub_euler = this->create_publisher<geometry_msgs::msg::Vector3>("mobictl/euler", 10);

  this->sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("web/joy", 10, std::bind(&MobiCtl::joy_callback, this, _1));
  this->sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MobiCtl::cmd_vel_callback, this, _1));

  this->declare_parameter("port", "/dev/ttyACM0");
  RCLCPP_DEBUG(this->get_logger(), "Serial port: %s", this->get_parameter("port").as_string().c_str());

  timer_ = this->create_wall_timer(std::chrono::milliseconds(0), std::bind(&MobiCtl::loop, this));
}

void MobiCtl::setup() {
  // auto port_name = this->get_first_serial_port_name();
  // if (port_name.empty()) {
  //   RCLCPP_INFO(this->get_logger(), "No serialport connected.");
  //   return;
  // };

  // this->serial_port.Open(port_name);
  this->serial_port.Open(this->get_parameter("port").as_string());
  this->serial_port.SetBaudRate(BaudRate::BAUD_115200);
  this->is_serial_connected = true;

  min_init_context(&min_ctx, 0);  // Init min proto
  min_transport_reset(&min_ctx, true);

  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::DISABLE_ALL_INTERVALS), {}, 0);  // Reser all Intervalls

  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::FIRMWARE_INFO), {}, 0);

  // Init Ulta
  PayloadBuilder *pb = new PayloadBuilder();
  pb->append_uint8(0b00111111);
  pb->append_uint16(1500);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::ULTRASONIC_SENSOR), pb->get_payload(), pb->size());

  // Init LUX
  pb->clear();
  pb->append_uint16(1500);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::BRIGHTNESS), pb->get_payload(), pb->size());

  // init temp
  pb->clear();
  pb->append_uint16(1500);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::TEMPERATURE), pb->get_payload(), pb->size());

  // Init IMU
  pb->clear();
  pb->append_uint8(0b01011100);
  pb->append_uint16(1500);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::IMU), pb->get_payload(), pb->size());

  // Init bat
  pb->clear();
  pb->append_uint16(10000);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::BAT_VOLTAGE), pb->get_payload(), pb->size());

  delete pb;
}

void MobiCtl::shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutting down MobiCtl");
  min_send_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::DISABLE_ALL_INTERVALS), {}, 0);  // Reser all Intervalls
  this->serial_port.Close();
}

bool lights = false;
uint8_t light_preset = 0;
uint8_t color_selected = 0;
std::vector<MobiCtl::COLOR_RGBW> colors = {
    {255, 0, 0, 0},  // red
    {0, 255, 0, 0},  // green
    {0, 0, 255, 0},  // blue
    {0, 0, 0, 255},
    {255, 255, 0, 0},
    {0, 255, 255, 0},
    {255, 0, 255, 0},
    {22, 48, 114, 0}  // FHWN
};

void MobiCtl::send_light_preset() {
  PayloadBuilder *pb = new PayloadBuilder();
  pb->append_uint8(light_preset);
  if (light_preset != 0) {
    pb->append_uint8(colors.data()[color_selected].r);
    pb->append_uint8(colors.data()[color_selected].g);
    pb->append_uint8(colors.data()[color_selected].b);
    pb->append_uint8(colors.data()[color_selected].w);
  }

  RCLCPP_INFO(this->get_logger(), "Setting color-preset %d with color %d, %d, %d, %d", light_preset, colors.data()[color_selected].r, colors.data()[color_selected].g, colors.data()[color_selected].b, colors.data()[color_selected].w);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::LED_STRIP), pb->get_payload(), pb->size());

  delete pb;
}

void MobiCtl::joy_callback(const sensor_msgs::msg::Joy &msg) {
  // Emergency stop
  if (msg.buttons[1] == 1) {
    // Handle Motor
    int16_t speed = 500;
    int16_t rot_speed = 2000;

    int16_t vy = speed * (msg.axes[5] - msg.axes[4]);
    int16_t vphi = rot_speed * (-msg.axes[0]);

    PayloadBuilder *pb = new PayloadBuilder();
    pb->append_int16(0);
    pb->append_int16(vy);
    pb->append_int16(vphi);
    min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::MOTOR_CONTROL), pb->get_payload(), pb->size());
    delete pb;
  } else {
    min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::MOTOR_CONTROL), {}, 0);
  }

  // B: Light on/off
  if (msg.buttons[0] == 1) {
    PayloadBuilder *pb = new PayloadBuilder();
    // Turn on Driving lights
    if (!lights) {
      pb->append_uint8(light_preset);
      lights = true;
    } else {
      lights = false;
    }

    min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::LED_STRIP), pb->get_payload(), pb->size());
    delete pb;
  }

  // Left: light_preset--
  if (msg.buttons[6] == 1) {
    if (light_preset == 0)
      light_preset = 3;
    else
      light_preset--;
    this->send_light_preset();
  }

  // Right: light_preset++
  if (msg.buttons[7] == 1) {
    light_preset++;
    if (light_preset > 3) light_preset = 0;
    this->send_light_preset();
  }

  // Up: color_selected++
  if (msg.buttons[4] == 1 && light_preset != 0) {
    color_selected++;
    if (color_selected >= colors.size()) color_selected = 0;
    this->send_light_preset();
  }

  // Down: color_selected--
  if (msg.buttons[5] == 1 && light_preset != 0) {
    if (color_selected == 0)
      color_selected = colors.size() - 1;
    else
      color_selected--;
    this->send_light_preset();
  }
}

void MobiCtl::cmd_vel_callback(const geometry_msgs::msg::Twist &msg) {
  // Handle Motor
  int16_t speed = 500;
  int16_t rot_speed = 2000;

  int16_t vx = speed * msg.linear.x;
  int16_t vy = speed * msg.linear.y;
  int16_t vphi = rot_speed * msg.angular.z;

  PayloadBuilder *pb = new PayloadBuilder();
  pb->append_int16(vy);  // axis are swapped because of robot coordinate system.
  pb->append_int16(vx);
  pb->append_int16(vphi);
  min_queue_frame(&min_ctx, static_cast<uint8_t>(COMMANDS::MOTOR_CONTROL), pb->get_payload(), pb->size());
  delete pb;
}

void MobiCtl::handle_min_frame(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload) {
  RCLCPP_DEBUG(this->get_logger(), "Handling MIN Frame with id %d and length %d", min_id, len_payload);

  switch (static_cast<DATA>(min_id)) {
    case DATA::COMMAND_STAUTS: {
      auto status = *min_payload;
      if (status != static_cast<uint8_t>(STATUS_CODE::OK)) RCLCPP_WARN(this->get_logger(), "Got Status Code: %d", status);
      break;
    }

    case DATA::IMU: {
      PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
      auto sub_device_masks = extract_subdevices_from_byte(pb->read_uint8());

      last_imu_msg.header.stamp = this->now();
      double w = pb->read_double();
      double x = pb->read_double();
      double y = pb->read_double();
      double z = pb->read_double();
      delete pb;

      switch (static_cast<IMU_SUB_DEVICES>(sub_device_masks.front())) {
        case IMU_SUB_DEVICES::ACCELEROMETER: {
          break;
        }
        case IMU_SUB_DEVICES::MAGNETOMETER: {
          break;
        }
        case IMU_SUB_DEVICES::GYROSCOPE: {
          geometry_msgs::msg::Vector3 vec3;
          vec3.x = x;
          vec3.y = y;
          vec3.z = z;
          last_imu_msg.angular_velocity = vec3;

          break;
        }
        case IMU_SUB_DEVICES::EULER: {
          geometry_msgs::msg::Vector3 vec3;
          vec3.x = x;
          vec3.y = y;
          vec3.z = z;
          this->pub_euler->publish(vec3);
          break;
        }
        case IMU_SUB_DEVICES::LINEAR_ACCEL: {
          geometry_msgs::msg::Vector3 vec3;
          vec3.x = x;
          vec3.y = y;
          vec3.z = z;
          last_imu_msg.linear_acceleration = vec3;

          break;
        }
        case IMU_SUB_DEVICES::GRAVITY: {
          break;
        }
        case IMU_SUB_DEVICES::QUATERNION: {
          geometry_msgs::msg::Quaternion quad;
          quad.w = w;
          quad.x = x;
          quad.y = y;
          quad.z = z;
          last_imu_msg.orientation = quad;
          break;
        }
      }

      this->pub_imu->publish(last_imu_msg);
      break;
    }

    case DATA::ULTRASONIC_SENSOR: {
      PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
      auto sub_device_masks = extract_subdevices_from_byte(pb->read_uint8());

      sensor_msgs::msg::Range msg;
      msg.range = pb->read_float();
      msg.min_range = 2.0;
      msg.max_range = 400.0;
      msg.radiation_type = 0;
      msg.header.stamp = this->now();
      delete pb;

      switch (static_cast<ULTRASONIC_SENSOR_SUB_DEVICES>(sub_device_masks.front())) {
        case ULTRASONIC_SENSOR_SUB_DEVICES::US_1: {
          this->pub_ultra_1->publish(msg);
          break;
        }

        case ULTRASONIC_SENSOR_SUB_DEVICES::US_2: {
          this->pub_ultra_2->publish(msg);
          break;
        }

        case ULTRASONIC_SENSOR_SUB_DEVICES::US_3: {
          this->pub_ultra_3->publish(msg);
          break;
        }

        case ULTRASONIC_SENSOR_SUB_DEVICES::US_4: {
          this->pub_ultra_4->publish(msg);
          break;
        }

        case ULTRASONIC_SENSOR_SUB_DEVICES::US_5: {
          this->pub_ultra_5->publish(msg);
          break;
        }

        case ULTRASONIC_SENSOR_SUB_DEVICES::US_6: {
          this->pub_ultra_6->publish(msg);
          break;
        }
      }

      break;
    }

    case DATA::ENCODER: {
      // PayloadBuilder *pb = new PayloadBuilder();
      // pb->append_uint8(sub_device.sub_device_mask);

      // switch (static_cast<ENCODER_SUB_DEVICES>(sub_device.sub_device_mask)) {
      //   case ENCODER_SUB_DEVICES::ENCODER_1: {
      //     pb->append_uint16(this->encoder_1->get_counter());
      //     break;
      //   }

      //   case ENCODER_SUB_DEVICES::ENCODER_2: {
      //     pb->append_uint16(this->encoder_2->get_counter());
      //     break;
      //   }

      //   case ENCODER_SUB_DEVICES::ENCODER_3: {
      //     pb->append_uint16(this->encoder_3->get_counter());
      //     break;
      //   }

      //   case ENCODER_SUB_DEVICES::ENCODER_4: {
      //     pb->append_uint16(this->encoder_4->get_counter());
      //     break;
      //   }
      // }

      // USB_COM_PORT::queue_payload(DATA::ENCODER, pb);
      // delete pb;
      break;
    }

    case DATA::BRIGHTNESS: {
      PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
      auto data = pb->read_float();
      delete pb;

      sensor_msgs::msg::Illuminance msg;
      std_msgs::msg::Header header;
      header.stamp = this->now();
      msg.header = header;
      msg.illuminance = data;
      this->pub_illuminance->publish(msg);
      break;
    }

    case DATA::TEMPERATURE: {
      PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
      auto data = pb->read_int8();
      delete pb;

      sensor_msgs::msg::Temperature msg;
      std_msgs::msg::Header header;
      header.stamp = this->now();
      msg.header = header;
      msg.temperature = static_cast<double>(data);
      this->pub_temperature->publish(msg);

      break;
    }

    case DATA::BAT_VOLTAGE: {
      PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
      auto data = pb->read_float();
      delete pb;

      sensor_msgs::msg::BatteryState msg;
      std_msgs::msg::Header header;
      header.stamp = this->now();
      msg.header = header;
      msg.voltage = data;
      msg.present = true;
      this->pub_battery->publish(msg);
      break;
    }

    case DATA::POZYX: {
      // PayloadBuilder *pb = new PayloadBuilder();
      // pb->append_uint8(sub_device.sub_device_mask);

      // uint8_t status = 0;  // Pozyx status

      // switch (static_cast<POZYX_SUB_DEVICES>(sub_device.sub_device_mask)) {
      //   case POZYX_SUB_DEVICES::POSITION: {
      //     // Read Pozyx position
      //     Pozyx::vector_t pos;
      //     status = this->pozyx->get_position(&pos);

      //     pb->append_vector(pos);
      //     break;
      //   }

      //   case POZYX_SUB_DEVICES::EULER: {
      //     // Read Pozyx euler
      //     Pozyx::vector_t euler;
      //     status = this->pozyx->get_euler(&euler);

      //     pb->append_vector(euler);
      //     break;
      //   }

      //   case POZYX_SUB_DEVICES::QUATERNION: {
      //     // Pozyx read quaternions
      //     Pozyx::vector_t quaternions;
      //     status = this->pozyx->get_quaternions(&quaternions);

      //     pb->append_vector(quaternions);
      //     break;
      //   }
      // }

      // if (status != 1) {
      //   debug_print("Error getting pozyx data, status: %d\n", status);
      //   this->send_status(STATUS_CODE::ERROR);
      // } else {
      //   USB_COM_PORT::queue_payload(DATA::POZYX, pb);
      // }
      // delete pb;
      break;
    }

    case DATA::POZYX_INFO: {
      // // Read Pozyx network id
      // uint16_t net_id;
      // uint8_t status = this->pozyx->get_network_id(&net_id);
      // if (status != 1) {
      //   debug_print("Error getting pozyx network id\n");
      //   this->send_status(STATUS_CODE::ERROR);
      //   break;
      // }

      // // Read Pozyx firmware version
      // uint8_t fw_version;
      // status = this->pozyx->get_firmware_version(&fw_version);
      // if (status != 1) {
      //   debug_print("Error getting pozyx firmware version\n");
      //   this->send_status(STATUS_CODE::ERROR);
      //   break;
      // }

      // // Read Pozyx hw version
      // uint8_t hw_version;
      // status = this->pozyx->get_harware_version(&hw_version);
      // if (status != 1) {
      //   debug_print("Error getting pozyx firmware version\n");
      //   this->send_status(STATUS_CODE::ERROR);
      //   break;
      // }

      // PayloadBuilder *pb = new PayloadBuilder();
      // pb->append_uint16(net_id);
      // pb->append_uint8(fw_version);
      // pb->append_uint8(hw_version);

      // USB_COM_PORT::queue_payload(DATA::POZYX_INFO, pb);
      // delete pb;
      break;
    }

    case DATA::FIRMWARE_INFO: {
      PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
      RCLCPP_INFO(this->get_logger(), "%s", pb->read_string().c_str());
      delete pb;
      break;
    }

    default:
      RCLCPP_INFO(this->get_logger(), "Got invalid MIN ID");
      break;
  }
}

std::vector<uint8_t> MobiCtl::extract_subdevices_from_byte(uint8_t byte) {
  std::vector<uint8_t> sub_devices;

  for (size_t i = 0; i < 8; i++) {
    // extract subdevice ids
    uint8_t current_mask = 0x01 << i;
    if ((byte & current_mask) == current_mask)
      sub_devices.push_back(current_mask);
  }

  return sub_devices;
}

std::string MobiCtl::get_first_serial_port_name() {
  auto ports = this->serial_port.GetAvailableSerialPorts();

#ifdef MOBICTL_DEBUG_PRINTING
  for (auto it = ports.begin(); it < ports.end(); ++it) {
    RCLCPP_DEBUG(this->get_logger(), "Available Port: %s", it->c_str());
  }
#endif

  if (!ports.empty()) {
    return ports.front();
  }
  return {};
}

void MobiCtl::loop() {
  if (!this->is_serial_connected) {
    // this->setup(); // FIXME: reconnect
    return;
  }

  try {
    auto read_count = this->serial_port.GetNumberOfBytesAvailable();

    DataBuffer data_buf;
    if (read_count == 0) {
      min_poll(&min_ctx, {}, 0);
      return;
    }
    this->serial_port.Read(data_buf, read_count, 100);
    min_poll(&min_ctx, data_buf.data(), data_buf.size());

  } catch (std::exception const &e) {
    if (this->is_serial_connected) {
      RCLCPP_WARN(this->get_logger(), "Serial Port Disconnected!");
      this->is_serial_connected = false;
    }
  }
}