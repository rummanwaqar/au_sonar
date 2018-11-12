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
 * The data is transmitted as 10-bit unsigned integers and should be converted to
 * voltage appropriately.
 *
 * Profiling (under 3% CPU)
 * 	Total processing of dataset = 10ms
 * 		Reading ping to vector = 6ms
 * 		Serialization = 3.5ms
 * 		Publishing data = 0.3ms
 */

#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <inttypes.h>
#include <signal.h>
#include <cstring>
#include <vector>
#include <ctime>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include <zmq.hpp>
#include <msgpack.hpp>

#include "shared_header.h"

//#define SERVER_ADDRESS "tcp://127.0.0.1:9999"
#define SERVER_ADDRESS "tcp://10.42.43.125:9999"

static int running = 1;

void sig_handler(int sig) {
	running = 0;
}

long int unix_timestamp()
{
    time_t t = std::time(0);
    long int now = static_cast<long int> (t);
    return now;
}

int main(int argc, char** argv) {
	// run as sudo (needed to access PRU systems)
	if (geteuid() != 0) {
		fprintf(stderr, "Must be root. Try again with sudo.\n");
		return EXIT_FAILURE;
	}

	// check command line arguments for PRU firmware files
	if (argc != 3) {
		fprintf(stderr, "Usage: %s pru0_code.bin pru1_code.bin\n", argv[0]);
		return EXIT_FAILURE;
	}

	// check if files exist
	if( access(argv[1], F_OK) == -1 ) {
		fprintf(stderr, "ERROR: %s does not exist. Specify the correct PRU0 binary\n", argv[1]);
		return EXIT_FAILURE;
	}
	if( access(argv[2], F_OK) == -1 ) {
		fprintf(stderr, "ERROR: %s does not exist. Specify the correct PRU1 binary\n", argv[2]);
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
	prussdrv_exec_program(0, argv[1]);
	prussdrv_exec_program(1, argv[2]);

	// start zmq publisher
	zmq::context_t context(1);
	zmq::socket_t publisher(context, ZMQ_PUB);
	publisher.bind(SERVER_ADDRESS);

	printf("Started ZMQ publisher at %s\n\n", SERVER_ADDRESS);

	// main loop
	while(running) {
		// wait for ping interrupt from PRU
		prussdrv_pru_wait_event(PRU_EVTOUT_1);

		std::vector<uint16_t> ping_data;

		// get read write pointers
		volatile uint32_t *read_pointer = shared_ddr;
		uint32_t *write_pointer_virtual = static_cast<uint32_t*>(prussdrv_get_virt_addr(pparams->shared_ptr));

		while(read_pointer != write_pointer_virtual) {
			// copy data to local memory
			uint16_t data[4];
			memcpy(data, (void*) read_pointer, 8);

			// use only first 10 bits that contain actual data
			data[2] &= 0x3ff;
			data[3] &= 0x3ff;

			// append to ping vector
			ping_data.insert(ping_data.end(), std::begin(data), std::end(data));

			// increment read pointer
			read_pointer += (8 / sizeof(*read_pointer));
		}
		printf(" [%ld] Got a ping and published %d data points.\n", unix_timestamp(), ping_data.size()/4);

		// serialize data
		msgpack::sbuffer serialized_buffer;
		msgpack::pack(serialized_buffer, ping_data);

		// publish ping data
	    zmq::message_t message(serialized_buffer.size());
	    memcpy(message.data(), serialized_buffer.data(), serialized_buffer.size());
	    publisher.send(message);

		// clear interrupt
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
	}

	printf("Quitting program\n");

	prussdrv_pru_disable(0);
	prussdrv_pru_disable(1);
	prussdrv_exit();

	return 0;
}
