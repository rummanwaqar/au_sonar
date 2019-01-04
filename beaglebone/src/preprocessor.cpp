

#include "preprocessor.hpp"

using namespace au_sonar;

Preprocessor::Preprocessor(std::string port) :
  serial_(port, 115200) {
    // set up callback function for serial reads
    serial_.register_receive_callback(std::bind(&Preprocessor::serial_callback,
      this, std::placeholders::_1, std::placeholders::_2));
}

bool Preprocessor::init() {
  // init serial
  if(!serial_.init()) {
    BOOST_LOG_TRIVIAL(fatal) << "Failed to initialize the serial port";
    return false;
  }
  return true;
}

void Preprocessor::serial_callback(const uint8_t* buf, size_t len) {
  std::string data((char*)buf, len);
  BOOST_LOG_TRIVIAL(debug) << data;
}
