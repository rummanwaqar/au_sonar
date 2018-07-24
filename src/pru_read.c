#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <signal.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "shared_header.h"

static int running = 1;

void sig_handler(int sig) {
	running = 0;
}

int main(int argc, char** argv) {
	// run as sudo (needed to access PRU systems)
	if (geteuid() != 0) {
		fprintf(stderr, "Must be root. Try again with sudo.\n");
		return EXIT_FAILURE;
	}

	// install signal handler
	if(SIG_ERR == signal(SIGINT, sig_handler)) {
		perror("Warn: signal handler not installed %d\n");
	}

	// initialize PRU and allocate memory
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_1);
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_pruintc_init(&pruss_intc_initdata);

	// get pointer into the shared PRU DRAM where PRU expects to share
	// params with PRUs and host CPU
	volatile pruparams_t *pparams = NULL;
	prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void**)&pparams);

	// pointer into the DDR RAM mapped by the uio_pruss kernel module. This
	// is host memory shared by to the PRU over OCP
	volatile uint32_t *shared_ddr = NULL;
	prussdrv_map_extmem((void**)&shared_ddr);
	unsigned int shared_ddr_len = prussdrv_extmem_size();
	unsigned int physical_addr = prussdrv_get_phys_addr((void*)shared_ddr);
	printf("%uKB of shared DDR available.\n Physical (PRU-side) address:%x\n Virtual (linux-side) address: %p\n\n",
			shared_ddr_len/1024, physical_addr, shared_ddr);

	// pass params to PRU
	pparams->physical_addr = physical_addr;
	pparams->ddr_len = shared_ddr_len;

	// load programs on to PRU units
	prussdrv_exec_program(0, "./pru0-clock.bin");
	prussdrv_exec_program(1, "./pru1-read-data.bin");

	// main loop
	while(running) {
		// wait for ping interrupt from PRU
		int n = prussdrv_pru_wait_event(PRU_EVTOUT_1);
		printf("got a ping\n");

		// clear interrupt
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
	}

	printf("Quitting program\n");

	prussdrv_pru_disable(0);
	prussdrv_pru_disable(1);
	prussdrv_exit();

	return 0;
}