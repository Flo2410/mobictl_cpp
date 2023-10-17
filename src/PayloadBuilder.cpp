#include "PayloadBuilder.hpp"

#include <memory>
#include "string"
#include "cstring"

PayloadBuilder::PayloadBuilder() {
}

PayloadBuilder::PayloadBuilder(uint8_t const* bytes, uint8_t length) {
  this->payload.insert(this->payload.begin(), bytes, bytes + length);
}

void PayloadBuilder::append_uint8(uint8_t number) {
  this->payload.push_back(number);
}

void PayloadBuilder::append_uint16(uint16_t number) {
  uint8_t buffer[sizeof(uint16_t)];
  std::memcpy(buffer, (uint8_t*)&number, sizeof(uint16_t));
  this->payload.insert(this->payload.end(), buffer, buffer + sizeof(uint16_t));
}

void PayloadBuilder::append_uint32(uint32_t number) {
  uint8_t buffer[sizeof(uint32_t)];
  std::memcpy(buffer, (uint8_t*)&number, sizeof(uint32_t));
  this->payload.insert(this->payload.end(), buffer, buffer + sizeof(uint32_t));
}

void PayloadBuilder::append_int16(int16_t number) {
  uint8_t buffer[sizeof(int16_t)];
  std::memcpy(buffer, (uint8_t*)&number, sizeof(int16_t));
  this->payload.insert(this->payload.end(), buffer, buffer + sizeof(int16_t));
}

void PayloadBuilder::append_float(float number) {
  uint8_t buffer[sizeof(float)];
  std::memcpy(buffer, (uint8_t*)&number, sizeof(float));
  this->payload.insert(this->payload.end(), buffer, buffer + sizeof(float));
}

void PayloadBuilder::append_double(double number) {
  uint8_t buffer[8];
  std::memcpy(buffer, (uint8_t*)&number, sizeof(double));
  this->payload.insert(this->payload.end(), buffer, buffer + sizeof(double));
}

void PayloadBuilder::append_string(std::string str) {
  this->payload.insert(this->payload.end(), str.c_str(), str.c_str() + str.length());
}

void PayloadBuilder::append_uint8_array(uint8_t * arr, uint8_t len) {
  this->payload.insert(this->payload.end(), arr, arr + len);
}

uint8_t PayloadBuilder::read_uint8() {
  uint8_t number = this->payload.front();
  this->payload.erase(this->payload.begin());
  return number;
}

int8_t PayloadBuilder::read_int8() {
  int8_t number = this->payload.front();
  this->payload.erase(this->payload.begin());
  return number;
}

uint16_t PayloadBuilder::read_uint16() {
  uint16_t number;
  std::memcpy(&number, this->payload.data(), sizeof(uint16_t));
  this->payload.erase(this->payload.begin(), this->payload.begin() + sizeof(uint16_t));
  return number;
}

int16_t PayloadBuilder::read_int16() {
  int16_t number;
  if (this->payload.size() < sizeof(int16_t)) return 0;
  std::memcpy(&number, this->payload.data(), sizeof(int16_t));
  this->payload.erase(this->payload.begin(), this->payload.begin() + sizeof(int16_t));
  return number;
}

float PayloadBuilder::read_float() {
  // if (index + sizeof(float) > buffer.size()) {
  //   // Handle error: Buffer size is not sufficient to read a float
  //   // You can throw an exception or return a default value based on your requirements.
  //   throw std::runtime_error("Buffer size is insufficient.");
  // }
  
  float number;
  std::memcpy((uint8_t*)&number, this->payload.data(), sizeof(float));
  this->payload.erase(this->payload.begin(), this->payload.begin() + sizeof(float));
  return number;
}


double PayloadBuilder::read_double() {
  double number;
  std::memcpy((uint8_t*)&number, this->payload.data(), sizeof(double));
  this->payload.erase(this->payload.begin(), this->payload.begin() + sizeof(double));
  return number;
}

std::string PayloadBuilder::read_string() {
  std::string str(this->payload.begin(), this->payload.end());
  this->payload.clear();
  return str;
}


uint8_t* PayloadBuilder::get_payload() {
  return this->payload.data();
}

size_t PayloadBuilder::size() {
  return this->payload.size();
}

void PayloadBuilder::clear() {
  this->payload.clear();
}

PayloadBuilder::~PayloadBuilder() {
  this->clear();
}