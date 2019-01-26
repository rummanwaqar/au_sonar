#include "pru_reader.hpp"

using namespace au_sonar;

PruReader::PruReader(const std::string pru0_fname, const std::string pru1_fname)
  : pru0_fname_(pru0_fname), pru1_fname_(pru1_fname), is_init_(false),
    pparams_(NULL), shared_ddr_(NULL) {}

PruReader::~PruReader() {
  pparams_ = NULL;
  shared_ddr_ = NULL;
  prussdrv_pru_disable(0);
  prussdrv_pru_disable(1);
  prussdrv_exit();
  LOG_INFO << "Closed PRU interface.";
}

bool PruReader::init() {
  if(is_init_) { // already initialized
    LOG_WARNING << "PRU Reader is already initialized.";
    return false;
  }

  // initialize PRU and allocate memory
  prussdrv_init();
  if(prussdrv_open(PRU_EVTOUT_1) != 0) {
    LOG_ERROR << "Could not open PRU Event 1.";
    return false;
  }
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  if(prussdrv_pruintc_init(&pruss_intc_initdata) != 0) {
    LOG_ERROR << "Failed to initialize PRU interrupt controller.";
    return false;
  }

  // get pointer into the shared PRU DRAM where PRU expects to share
  // params with PRUs and host CPU
  if(prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void **)&pparams_) != 0) {
    LOG_ERROR << "Failed to map the PRU memory.";
    return false;
  }

  // pointer into the DDR RAM mapped by the uio_pruss kernel module. This
  // is host memory shared by to the PRU over OCP
  prussdrv_map_extmem((void **)&shared_ddr_);
  unsigned int shared_ddr_len = prussdrv_extmem_size();
  unsigned int physical_addr = prussdrv_get_phys_addr((void *)shared_ddr_);
  LOG_INFO << shared_ddr_len / 1024 << "KB of shared DDR available.";
  LOG_INFO << "Physical (PRU-side) address: 0x" << std::hex << physical_addr;
  char formatted_shared_addr[100];
  sprintf(formatted_shared_addr, "%p", shared_ddr_);
  LOG_INFO << "Virtual (linux-side) address: " << formatted_shared_addr;

  // pass params to PRU
  pparams_->physical_addr = physical_addr;
  pparams_->ddr_len = shared_ddr_len;

  // load firmware
  if(prussdrv_exec_program(0, pru0_fname_.c_str()) == 0) {
    LOG_DEBUG << "PRU 0 loaded.";
  } else {
    LOG_ERROR << "Error loading PRU 0 firmware.";
    return false;
  }
  if(prussdrv_exec_program(1, pru1_fname_.c_str()) == 0) {
    LOG_DEBUG << "PRU 1 loaded.";
  } else {
    LOG_ERROR << "Error loading PRU 1 firmware.";
    return false;
  }

  // create a thread for io service to run on
  io_thread_ = std::thread([&]{ run(); });
  io_thread_.detach();

  return true;
}

static inline float to_voltage(int adc_sample) {
  return (float(adc_sample) / 1024.0) * 2.0;
}

void PruReader::run() {
  while(1) {
    // wait for ping interrupt from PRU
    prussdrv_pru_wait_event(PRU_EVTOUT_1);

    // hold ping data for a full ping (automatically sets timestamp at creation)
    PingData pingdata;

    // get read write pointers
    volatile uint32_t *read_pointer = shared_ddr_;
    uint32_t *write_pointer_virtual =
        static_cast<uint32_t *>(prussdrv_get_virt_addr(pparams_->shared_ptr));

    while (read_pointer != write_pointer_virtual) {
      // copy data to local memory
      uint16_t data[4];
      memcpy(data, (void *)read_pointer, 8);
      /* hydrophone data is only the first 10 bits.
       * channel 2 hydrophones data[2] and data[3] contain come data
       * in MSBs that is used in PRUs and should be masked off */
      data[2] &= 0x3ff;
      data[3] &= 0x3ff;

      AdcSample sample;
      sample.a = to_voltage(data[0]);
      sample.refa = to_voltage(data[1]);
      sample.b = to_voltage(data[2]);
      sample.refb = to_voltage(data[3]);

      // append to ping vector
      pingdata.data.push_back(sample);

      // increment read pointer
      read_pointer += (8 / sizeof(*read_pointer));
    }
    LOG_INFO << pingdata.to_string();

    // clear interrupt
    prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
  }
}

float to_voltage(const int16_t) {
  return 0;
}
