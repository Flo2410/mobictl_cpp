#ifndef PAYLOAD_BUILDER_H_
#define PAYLOAD_BUILDER_H_

#include <memory>

#include "string.h"
#include "vector"

class PayloadBuilder {
 public:
  PayloadBuilder();
  PayloadBuilder(uint8_t const* bytes, uint8_t length);
  ~PayloadBuilder();

  void append_uint8(uint8_t number);
  void append_uint16(uint16_t number);
  void append_uint32(uint32_t number);
  void append_int16(int16_t number);
  void append_float(float number);
  void append_double(double number);
  void append_string(std::string str);

  void append_uint8_array(uint8_t* arr, uint8_t len);

  uint8_t read_uint8();
  int8_t read_int8();
  uint16_t read_uint16();
  int16_t read_int16();
  float read_float();
  double read_double();
  std::string read_string();

  uint8_t* get_payload();
  size_t size();
  void clear();

 private:
  std::vector<uint8_t> payload;
};

#endif