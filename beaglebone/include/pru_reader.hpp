/*
 * @author: Rumman Waqar
 *
 * Runs an async process that retrieves PRU data and publishes it
 */

#ifndef _AU_SONAR_PRU_READER_H_
#define _AU_SONAR_PRU_READER_H_

#include <string>
#include <thread>
#include <cstdio>

#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <plog/Log.h>

#include "shared_header.h"
#include "datatypes.hpp"

namespace au_sonar {
  class PruReader {
  public:
    /*
     * initalize PRU object with pru firmware filenames and sonar_data object
     */
    PruReader(const std::string pru0_fname, const std::string pru1_fname,
      SonarData& sonar_data);

    /*
     * closes connection to PRU and cleans up
     */
    ~PruReader();

    /*
     * initializes the PRU event handler, loads firmware and starts processing thread
     * returns false if init fails
     */
    bool init();

  private:
    /*
     * async task run on a seperate thread. Waits for PRU ADC data, packages it
     * and sends it to sonardata object
     */
    void run();

    // filenames for PRU firmwares
    const std::string pru0_fname_;
    const std::string pru1_fname_;

    // is PRU unit initialized
    bool is_init_;

    // thread for PRU data acquisition
    std::thread io_thread_;

    // pointer into the DDR RAM mapped by the uio_pruss kernel module. This
    // is host memory shared by to the PRU over OCP
    volatile uint32_t* shared_ddr_;

    // get pointer into the shared PRU DRAM where PRU expects to share
    // params with PRUs and host CPU
    volatile pruparams_t* pparams_;

    // sonar data reference. manages sync between sonar info and data
    SonarData& sonar_data_; 
  }; // class PruReader
} // namespace au_sonar

#endif
