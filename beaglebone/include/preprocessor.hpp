/*
 * @author: Rumman Waqar
 *
 * Preprocessor class handles the high level communication for preprocessor
 * board
 */

#ifndef _AU_SONAR_PREPROCESSOR_H_
#define _AU_SONAR_PREPROCESSOR_H_

#include <functional>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <plog/Log.h>
#include <boost/algorithm/string.hpp>

#include "datatypes.hpp"
#include "serial.hpp"

namespace au_sonar {
class Preprocessor {
 public:
  /*
   * create serial object and register callback
   */
  Preprocessor(std::string port, SonarData& sonar_data);

  /*
   * initialize serial object
   */
  bool init();

  /*
   * send a commmand to preprocessor board using serial
   * returns result
   * throws runtime_error exception if unable to verify without timeout
   * (milliseconds) command adds newline
   */
  std::string write_command(const std::string& command);

 private:
  /*
   * callback for serial reads
   */
  void serial_callback(const uint8_t* buf, size_t len);

  /*
   * parse input to a ping or read/write response
   * if ping, added to queue
   */
  void parse_input(std::string&& line);

  au_sonar::Serial serial_;
  std::string
      buffer_;  // buffer to store incoming data until a complete line is found
  std::queue<PingInfo> ping_queue_;  // stores incoming ping status messages
  au_sonar::SerialResponse response_;

  SonarData& sonar_data_;  // sonar data reference. manages sync between sonar
                           // info and data
};
}  // namespace au_sonar

#endif
