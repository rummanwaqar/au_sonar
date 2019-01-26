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
#include <thread>

#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <plog/Log.h>

#include "shared_header.h"
#include "datatypes.hpp"

namespace au_sonar {
  class PruReader {
  public:
    PruReader(const std::string pru0_fname, const std::string pru1_fname,
      SonarData& sonar_data);

    ~PruReader();

    bool init();

  private:
    void run();

    const std::string pru0_fname_;
    const std::string pru1_fname_;
    bool is_init_;
    std::thread io_thread_;

    volatile uint32_t* shared_ddr_;
    volatile pruparams_t* pparams_;

    SonarData& sonar_data_; // sonar data reference. manages sync between sonar info and data


  }; // class PruReader
} // namespace au_sonar

#endif
