#include "preprocessor.hpp"
#include <iostream>

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
  std::string raw_data((char*)buf, len);

  // find new line
  std::size_t n_index = raw_data.find('\n');
  if (n_index!=std::string::npos) {
    // line = old data + everything up to but not including \n
    std::string line = buffer_ + raw_data.substr(0, n_index);
    // put the remainder in the buffer
    buffer_ = raw_data.substr(n_index+1, std::string::npos);
    // process line
    parse_input(std::move(line));
  } else {
    // append all to buffer
    buffer_ += raw_data;
  }

  if(buffer_.size() > 5000) {
    // buffer size should never get too big. Reset it
    buffer_.clear();
    BOOST_LOG_TRIVIAL(warning) << "Preprocessor input buffer got too big and had to be cleared.";
  }
}

void Preprocessor::parse_input(std::string&& line) {
  // find the starting character $
  std::size_t starting_index = line.find('$');
  if(starting_index!=std::string::npos) { // starting index found
    line = line.substr(starting_index, std::string::npos);
    BOOST_LOG_TRIVIAL(debug) << line;
  } else { // invalid line
    BOOST_LOG_TRIVIAL(warning) << "Preprocessor input line invalid with no $ starting character ("
      << line << ")";
  }
}
