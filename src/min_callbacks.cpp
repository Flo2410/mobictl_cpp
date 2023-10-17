#include "MobiCtl.hpp"
#include "min.h"
#include "rclcpp/rclcpp.hpp"
#include "string"

#ifdef MIN_DEBUG_PRINTING

void min_debug_print(const char* format, ...) {
  auto logger = rclcpp::get_logger("MIN");
  rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  // Strip newline if present
  std::string message = buffer;
  if (!message.empty() && message.back() == '\n') {
    message.pop_back();
  }

  RCLCPP_DEBUG(logger, "%s", message.c_str());
}
#endif

//--------------------------------------------------------------------------------------------------------------
// MIN Callback functions
//--------------------------------------------------------------------------------------------------------------
void min_tx_start(uint8_t /*port*/) {
  // USB_COM_PORT::tx_queue.clear();
  // std::queue<uint8_t> empty;
  // std::swap(tx_queue, empty);
  MobiCtl::mobictl()->tx_queue.clear();
}

void min_tx_finished(uint8_t /*port*/) {
  // USB_COM_PORT::send_data(&USB_COM_PORT::tx_queue.front(), USB_COM_PORT::tx_queue.size());
  std::string str("min_tx_finished with tx_queue: ");
  // std::string result(MobiCtl::mobictl()->tx_queue.begin(), MobiCtl::mobictl()->tx_queue.end());

  std::ostringstream oss;
  oss << std::hex << std::setfill('0');

  for (const auto& byte : MobiCtl::mobictl()->tx_queue) {
    oss << "0x" << std::setw(2) << static_cast<int>(byte) << " ";
  }

  std::string hexString = oss.str();

  str.append(hexString);
  min_debug_print(str.c_str());

  MobiCtl::mobictl()->serial_port.Write(MobiCtl::mobictl()->tx_queue);
}

// Tell MIN how much space there is to write to the serial port. This is used
// inside MIN to decide whether to bother sending a frame or not.
uint16_t min_tx_space(uint8_t /*port*/) {
  // Ignore 'port' because we have just one context. But in a bigger application
  // with multiple ports we could make an array indexed by port to select the serial
  // port we need to use.
  // return USB_COM_PORT::tx_queue.available();
  return MobiCtl::mobictl()->tx_queue.max_size() - MobiCtl::mobictl()->tx_queue.size();
}

// Send a character on the designated port.
void min_tx_byte(uint8_t /*port*/, uint8_t byte) {
  // Ignore 'port' because we have just one context.
  // USB_COM_PORT::tx_queue.push(byte);
  MobiCtl::mobictl()->tx_queue.push_back(byte);
}

// Tell MIN the current time in milliseconds.
uint32_t min_time_ms(void) {
  // return HAL_GetTick();
  // namespace sc = std::chrono;
  // return sc::duration_cast<sc::milliseconds>(sc::system_clock::now().time_since_epoch());
  return MobiCtl::mobictl()->get_clock()->now().nanoseconds() * 1000;
  // return 1000;
}

// Handle the reception of a MIN frame. This is the main interface to MIN for receiving
// frames. It's called whenever a valid frame has been received (for transport layer frames
// duplicates will have been eliminated).
void min_application_handler(
    uint8_t min_id, uint8_t const* min_payload, uint8_t len_payload,
    uint8_t /*port*/) {
  // We ignore the port because we have one context, but we could use it to index an array of
  // contexts in a bigger application.

  // printf("MIN frame with ID 0x%02x, length = %d and data: %s\n", min_id, len_payload, (char *)min_payload);
  // RCLCPP_INFO(node->get_logger(), "MIN frame with ID 0x%02x, length = %d and data: %s\n", min_id, len_payload, (char *)min_payload);

  // Handle Frames
  MobiCtl::mobictl()->handle_min_frame(min_id, min_payload, len_payload);
}
