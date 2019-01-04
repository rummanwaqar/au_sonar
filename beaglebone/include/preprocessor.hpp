/*
 * @author: Rumman Waqar
 *
 * Preprocessor class handles the high level communication for preprocessor board
 */

#ifndef _AU_SONAR_PREPROCESSOR_H_
#define _AU_SONAR_PREPROCESSOR_H_

#include <string>
#include <functional>
#include <boost/log/trivial.hpp>

#include "serial.hpp"

namespace au_sonar {
  class Preprocessor {
  public:
    /*
     * create serial object and register callback
     */
    Preprocessor(std::string port);

    /*
     * initialize serial object
     */
    bool init();

  private:
    /*
     * callback for serial reads
     */
    void serial_callback(const uint8_t* buf, size_t len);

    au_sonar::Serial serial_;
  };
} // namespace au_sonar

#endif
