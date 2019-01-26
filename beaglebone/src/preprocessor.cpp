#include "preprocessor.hpp"
using namespace au_sonar;

Preprocessor::Preprocessor(std::string port, SonarData& sonar_data)
    : serial_(port, 115200), sonar_data_(sonar_data) {
  // set up callback function for serial reads
  serial_.register_receive_callback(std::bind(&Preprocessor::serial_callback,
                                              this, std::placeholders::_1,
                                              std::placeholders::_2));
}

bool Preprocessor::init() {
  // init serial
  if (!serial_.init()) {
    LOG_FATAL << "Failed to initialize the serial port";
    return false;
  }
  return true;
}

std::string Preprocessor::write_command(const std::string& command) {
  std::string command_with_nl = command + "\n";
  if (serial_.write(command_with_nl) != command_with_nl.size()) {
    throw std::runtime_error("Failed to write command: " + command);
  }
  // process echoed data
  std::string output = response_.get_data(20);
  if (output == "") {
    return "Failed to get response for command: " + command;
  }
  return output;
}

void Preprocessor::serial_callback(const uint8_t* buf, size_t len) {
  std::string raw_data((char*)buf, len);

  // find new line
  std::size_t n_index = raw_data.find('\n');
  if (n_index != std::string::npos) {
    // line = old data + everything up to but not including \n
    std::string line = buffer_ + raw_data.substr(0, n_index);
    // put the remainder in the buffer
    buffer_ = raw_data.substr(n_index + 1, std::string::npos);
    // process line
    parse_input(std::move(line));
  } else {
    // append all to buffer
    buffer_ += raw_data;
  }

  if (buffer_.size() > 5000) {
    // buffer size should never get too big. Reset it
    buffer_.clear();
    LOG_WARNING
        << "Preprocessor input buffer got too big and had to be cleared.";
  }
}

void Preprocessor::parse_input(std::string&& line) {
  if (line.size() < 1) {
    return;
  }
  // find the starting character $
  std::size_t starting_index = line.find('$');
  if (starting_index != std::string::npos) {  // starting index found
    line = line.substr(starting_index + 1, std::string::npos);
    // removing last char because its carriage return
    line = line.erase(line.size() - 1);
    // split line over spaces
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));

    if (tokens[0] == "ping") {  // ping data
      // build ping info object
      PingInfo info;
      for_each(tokens.begin() + 1, tokens.end(), [&](std::string const& token) {
        std::vector<std::string> pair;
        boost::split(pair, token, boost::is_any_of("="));
        info.data[pair[0]] = std::stof(pair[1]);
      });
      LOG_INFO << info.to_string();
      // add info to sonar data object
      sonar_data_.add_data(std::move(info));
    } else {  // read/write response
      // set data
      response_.add_data(std::move(line));
      LOG_VERBOSE << "Write reponse: " << line;
    }
    LOG_DEBUG << line;
  } else {  // invalid line
    LOG_WARNING
        << "Preprocessor input line invalid with no $ starting character ("
        << line << ")";
  }
}
