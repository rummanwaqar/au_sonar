/*
 * @author: Rumman Waqar
 *
 * Runs an async process that retrieves PRU data and publishes it
 */

 /*
  * Author: Rumman Waqar
  *
  * Runs a ZMQ publisher that publishes sonar data every time it receives a ping.
  * The program takes in URL to PRU firmwares as arguments.
  *
  * The output data format is an array with n elements where n%4 == 0.
  * The format of data output is [hydrophoneA, refA, hydrophoneB, refB, ... ] and
  * the pattern repeats for n/4 times.
  *
  * The data is transmitted as 10-bit unsigned integers and should be converted
  * to voltage appropriately.
  *
  * Profiling (under 3% CPU)
  * 	Total processing of dataset = 10ms
  * 		Reading ping to vector = 6ms
  * 		Serialization = 3.5ms
  * 		Publishing data = 0.3ms
  */

#ifndef _AU_SONAR_PRU_READER_H_
#define _AU_SONAR_PRU_READER_H_

#include <string>

#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <plog/Log.h>

#include "shared_header.h"

namespace au_sonar {
  class PruReader {
  public:
    PruReader(const std::string pru0_fname, const std::string pru1_fname);

    ~PruReader();

    bool init();

  private:
    const std::string pru0_fname_;
    const std::string pru1_fname_;
    bool is_init_;

  }; // class PruReader
} // namespace au_sonar

#endif
