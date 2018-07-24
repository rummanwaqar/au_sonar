// -*- mode: asm -*-

/*
 * This code runs on PRU1 and retrieves the samples generated as PRU0
 * switches back and forth between input 0(A) and 1(B) + 4(REF)
 *
 * The system sends an interrupt (PRU_EVTOUT_1) after a ping data has
 * been collected.
 *
 * Register 31: (inputs)
 *  Bit 0-9: ADC data out
 *  Bit 10: INPUT0A status
 *  Bit 16: INPUT0B status
 *  Bit 11: ADC clock status
 *  Bit 13: ping signal
 *
 */
 
 .origin 0
 .entrypoint TOP
 
 #include <shared_header.h>
 
 #define SHARED_RAM_ADDRESS 0x10000
 #define PRU1_R31_VEC_VALID 32
 #define PRU_EVTOUT_1 4
 
 // scratch pad registers
 #define DATA_START r15		// start register of input to write
 #define DATA_LEN	8		// write 2 32-bit registers (input0 and input1)
 #define INPUT0_REG	r15.w0	// stores value for input0 in bottom half of register
 #define INPUT0_REF	r15.w2	// stores value for input0 reference in top half of register
 #define INPUT1_REG	r16.w0	// stores value for input1 in bottom half of register
 #define INPUT1_REF	r16.w2	// stores value for input1 reference in top half of register
 #define QUEUE_PTR	r19		// current write pointer in queue
 #define QUEUE_END	r20		// max end pointer of queue
 #define SAMPLE_REG	r22		// read ADC data to this register temporarily
 #define SHARED_RAM r27		// shared PRU memory address
 #define DDR_SIZE	r28		// global memory size
 #define DDR		r29		// global memory address
 #define MASK_REG	r12		// mask that lets us exlude unused GPIO
 
 #define nop add r0, r0, 0
 
 TOP:
 	// enable OCP master ports in SYSCFG register
 	lbco r0, C4, 4, 4
 	clr  r0, r0, 4
 	sbco r0, C4, 4, 4
 	
 	// load address of PRU shared RAM in register
 	mov SHARED_RAM, SHARED_RAM_ADDRESS
 	// get address of shared global memory from shared RAM
 	lbbo DDR, SHARED_RAM, OFFSET(Params.physical_addr), SIZE(Params.physical_addr)
 	// get size of shared global memory from shared RAM
 	lbbo DDR_SIZE, SHARED_RAM, OFFSET(Params.ddr_len), SIZE(Params.ddr_len)
 	
 	// mask for incoming data. 10 bits of data. 
 	// Bit on 10 will tell us which input is selected (INPUT0B)
 	mov MASK_REG, 0x000007ff
 	
 	// initialize queue ptrs
 	mov QUEUE_PTR, DDR
 	add QUEUE_END, DDR, DDR_SIZE
 	
 	// if ping signal is low wait for it to get high
 	qbbc WAIT_PING_NO_INTERRUPT, r31.t13
 	
 MAIN_LOOP:
 	// update the location of the write pointer so the host CPU can see it. (2 cycles)
 	sbbo QUEUE_PTR, SHARED_RAM, OFFSET(Params.shared_ptr), SIZE(Params.shared_ptr)
 	
 	// if adc line is low wait for it to go high
 	qbbc WAIT_FOR_HIGH, r31.t11
 ADC_HIGH:
 	// adc line is high. It will be about 11ns before the data is valid
 	nop
 	nop
 	nop
 	// read and mask GPIO. This will be hydrophone A or B (input 0 or 1)
 	and SAMPLE_REG, r31, MASK_REG
 	
 	// if bit 10 is set the sample corresponds with input 1 (hydrophone B)
 	qbbs CHANNEL0_SECOND_INPUT, SAMPLE_REG, 10
 CHANNEL0_FIRST_INPUT:
 	// INPUT0_REG = SAMPLE_REG
 	mov INPUT0_REG, SAMPLE_REG
 	qba CHANNEL0_DONE
 CHANNEL0_SECOND_INPUT:
 	// INPUT1_REG = SAMPLE_REG
 	mov INPUT1_REG, SAMPLE_REG
 CHANNEL0_DONE:
 	
 	// if adc line is still high, wait for it to go low
 	qbbs WAIT_FOR_LOW, r31.t11
 ADC_LOW:
 	// adc line is low. It will be about 11ns before the data is valid
 	nop
 	nop
 	nop
 	// read and mask GPIO. This will be reference hydrophone (input 4)
 	and SAMPLE_REG, r31, MASK_REG
 	qbbs CHANNEL1_SECOND_INPUT, SAMPLE_REG, 10
 CHANNEL1_FIRST_INPUT:
 	// INPUT0_REF = SAMPLE_REG
 	mov INPUT0_REF, SAMPLE_REG
 	qba CHANNEL1_DONE
 CHANNEL1_SECOND_INPUT:
 	// INPUT1_REF = SAMPLE_REG
 	mov INPUT1_REF, SAMPLE_REG
 	qba WRITE_MEMORY
 CHANNEL1_DONE:
 		
 	// if ping signal is low wait for it to get high and send interrupt to read all collected data
	qbbc WAIT_PING, r31.t13
	
	// loop back to start
	qba MAIN_LOOP
 
 WRITE_MEMORY:
 	// write both pairs to global memory
 	sbbo DATA_START, QUEUE_PTR, 0, DATA_LEN
 	// increment queue write ptr
 	add QUEUE_PTR, QUEUE_PTR, DATA_LEN
 	
 	// if queue is not overflowing return back to main loop
 	qblt CHANNEL1_DONE, QUEUE_END, QUEUE_PTR
 	// else reset the queue and then return to main loop
 	mov QUEUE_PTR, DDR
 	qba CHANNEL1_DONE	
 
 
 WAIT_FOR_HIGH:
 	// if ping signal is clear (no ping) wait for ping to become available
 	qbbc WAIT_PING, r31.t13
 	// if adc bit is low, continue looping
 	qbbc WAIT_FOR_HIGH, r31.t11
 	// once adc bit goes high go back to main loop
 	qba  ADC_HIGH
 	
 WAIT_FOR_LOW:
 	// if ping signal is clear (no ping) wait for ping to become available
 	qbbc WAIT_PING, r31.t13
 	// if adc bit is high, continue looping
 	qbbs WAIT_FOR_LOW, r31.t11
 	// one adc bit goes low go back to main loop
 	qba  ADC_LOW
 	
 WAIT_PING:
 	// interrupt the host
 	mov  r31.b0, PRU1_R31_VEC_VALID | PRU_EVTOUT_1
 WAIT_PING_NO_INTERRUPT:
	// wait until bit is set
	wbs r31.t13
	// reset the queue
	mov QUEUE_PTR, DDR
	qba MAIN_LOOP
	