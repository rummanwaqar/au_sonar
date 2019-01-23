/*
 * @author: Rumman Waqar
 *
 * Preprocessor class handles the high level communication for preprocessor board
 */

#ifndef _AU_SONAR_PREPROCESSOR_H_
#define _AU_SONAR_PREPROCESSOR_H_

#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <functional>
#include <boost/log/trivial.hpp>
#include <boost/algorithm/string.hpp>

#include "serial.hpp"
#include "datatypes.hpp"

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
     * throws runtime_error exception if unable to verify without timeout (milliseconds)
     * command adds newline
     */
    std::string write_command(std::string command, int timeout);

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
    std::string buffer_; // buffer to store incoming data until a complete line is found
    std::queue<PingInfo> ping_queue_; // stores incoming ping status messages

    SonarData& sonar_data_; // sonar data reference. manages sync between sonar info and data
  };
} // namespace au_sonar

#endif
