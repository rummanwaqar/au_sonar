/*
 * This file contains structure definitions for sharing parameters
 * between the PRUs and the host processor.
 *
 * BUILD_WITH_PASM must be defined as an option when building .p files
 */

#ifndef BUILD_WITH_PASM

typedef struct {
	uint32_t physical_addr;
	uint32_t ddr_len;
	uint32_t shared_ptr;
} pruparams_t;

#else

.struct Params
	.u32 physical_addr
	.u32 ddr_len
	.u32 shared_ptr
.ends

#endif
