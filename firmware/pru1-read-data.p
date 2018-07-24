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
 *  Bit 10: INPUT0B status
 *  Bit 16: INPUT0A status
 *  Bit 11: ADC clock status
 *  Bit 13: ping signal
 *
 */
 
 .origin 0
 .entrypoint TOP
 
 #define PRU1_R31_VEC_VALID 32
 #define PRU_EVTOUT_1 4
 
 TOP:
 	// enable OCP master ports in SYSCFG register
 	lbco r0, C4, 4, 4
 	clr  r0, r0, 4
 	sbco r0, C4, 4, 4
 	
 	//mov SHARED_RAM, SHARED_RAM_ADDRESS
 	
 	// if ping signal is low wait for it to get high
 	qbbc WAIT_PING_NO_INTERRUPT, r31.t13
 	
 MAIN_LOOP:
 	// if ping signal is low wait for it to get high and send interrupt to read all collected data
	qbbc WAIT_PING, r31.t13
	
	// loop back to start
	qba MAIN_LOOP
 	
 WAIT_PING:
 	// interrupt the host
 	mov  r31.b0, PRU1_R31_VEC_VALID | PRU_EVTOUT_1
 WAIT_PING_NO_INTERRUPT:
	// wait until bit is set
	wbs r31.t13
	qba MAIN_LOOP
	