/*
 * This file contains structure definitions for sharing parameters
 * between the PRUs and the host processor.
 *
 * BUILD_WITH_PASM must be defined as an option when building .p files
 */

#ifndef BUILD_WITH_PASM

typedef struct {
  // Physical address of the start of the shared main memory buffer.
  // (The PRUs don't go through the virtual memory system, so they
  // see different memory addresses than the linux side does).
  // Written by the CPU, read by the PRU
  uint32_t physical_addr;

  // Length in bytes of the shared main memory buffer
  // Written by the CPU, read by the PRU
  uint32_t ddr_len;

  // Physical address of where the PRU is going to write next.
  // Written by the PRU, read by the CPU
  uint32_t shared_ptr;
} pruparams_t;

#else

// clang-format off
.struct Params
  .u32 physical_addr
  .u32 ddr_len
  .u32 shared_ptr
.ends
// clang-format on

#endif
