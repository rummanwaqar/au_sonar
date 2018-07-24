// -*- mode: asm -*-

/*
 * This code runs on PRU0. It generates a GPIO clock while alternating the 
 * analog switches to allow round robin sample on channel 0. The signals are
 * only generated when ping is available.
 * 
 * PRU clock is 200MHz = 5ns 
 * ADC clock runs at 2MHz (1MSPS on each input pair) 
 * 	no.of cycles = (PRU_CLOCK/SAMPLE_FREQUENCY) = 200MHz/2MHz = 100 cycles
 *  no.of half cycles = 50 cycles
 * 
 * Register 30: (outputs)
 *	Bit 0: clock
 *  Bit 5 and 3: channel 0 input select (00=input_0, 01=input_1)
 *  Bit 1 and 2: channel 1 input select (00=input_4)
 *  Bit 7: ADC disable (pull low to enable)
 *
 * hydrophone_A = input_0
 * hydrophone_B = input_1
 * hydrophone_base = input_4
 *
 * Register 31: (inputs)
 *  Bit 14: ping signal
 *
 * We need to switch channels on the analog muxes before we sample,
 * and the MAX4734 datasheet says that takes 25ns. The AD9201 has 4ns 
 * aperture delay.  So we have to wait that long after the rising clock
 * edge before we switch. The input RC filter (10 ohms + 100pF) has a 
 * corner at 160MHz, so a bit less than 10ns. So we want to switch at 
 * least 35ns before we sample, and after we sample
 * we shouldn't switch for 4ns.  
 */
 
.origin 0
.entrypoint TOP

#define HALF_CYCLE_COUNT 50

#define PAUSE_COUNT r22

// do nothing
.macro NOP
	add r0, r0, 0
.endm

// pause for n cycles. n must be >= 6
.macro PAUSE
.mparam count
	mov	 PAUSE_COUNT, count
	sub	 PAUSE_COUNT, PAUSE_COUNT, 3		// subtract 3 cycles for initialization
	lsr  PAUSE_COUNT, PAUSE_COUNT, 1		// divide by 2 because each pause loop is two instructions
PAUSE_LOOP:
	sub  PAUSE_COUNT, PAUSE_COUNT, 1
	qblt PAUSE_LOOP, PAUSE_COUNT, 0
.endm


// entrypoint
TOP:
	// Enable OCP master ports in SYSCFG register
	lbco r0, C4, 4, 4
	clr	 r0, r0, 4
	sbco r0, C4, 4, 4
		
	// clock low, input=0,4 (also enables ADC by pulling bit 7 low)
	mov r30, 0x00
	// wait half cycle (minus 1 for the mov command above)
	PAUSE HALF_CYCLE_COUNT - 1
	
REPEAT:
	// clock goes high, inputs=0,4
	mov r30, 0x01
	
	// wait 10ns for aperture delay
	NOP
	NOP
	
	// switch mux to input 1
	mov r30, (1 << 3) | 0x01
	
	// pause before switching to low again (minus 4 for the 4 commands above)
	PAUSE HALF_CYCLE_COUNT - 4
	
	// clock goes low, inputs=1,4
	mov r30, (1 << 3) | 0x00
	
	// pause before switching to high again
	PAUSE HALF_CYCLE_COUNT - 1
	
	// clock goes high, inputs=1,4
	mov r30, (1 << 3) | 0x01
	
	// wait 10ns for aperture delay
	NOP
	NOP
	
	// switch back to inputs=0,4 (clock stays high)
	mov r30, 0x01
	
	// pause before switching to low again
	PAUSE HALF_CYCLE_COUNT - 4
	
	// clock goes low, inputs=0,4
	mov r30, 0x00
	
	// if ping signal is low wait for it to get high
	qbbc WAIT_PING, r31.t14
	
	// pause before switching to high again (minus 3 for two commands before and 1 after)
	PAUSE HALF_CYCLE_COUNT - 3
	
	// loop back to start
	qba REPEAT
	
	// never gets here (exit)
	HALT
	
WAIT_PING:
	// wait until bit is set
	wbs r31.t14
	qba REPEAT
	