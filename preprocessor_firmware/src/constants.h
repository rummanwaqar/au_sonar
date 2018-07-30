#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

//Initalizing constants for the gain controller
#define DEFAULT_GAIN 40  //The gain the amplifier starts at in dB
#define DESIRED_PEAK 1.0 //The intial target output in Vpp.
#define DC_BIAS 0.97  //Peak Detector reading without any input to the sonar board
//Initial gains for the controller
#define P_GAIN 0.1   //The error at the moment of getting the peak value.
#define I_GAIN 0.01  //Sums the error over time
#define MAX_I_ERROR 0.5 //Saturates the integral error.
//Initial durations in milliseconds
#define FLOOR_GAIN_DURATION 800  //How long to floor gain for after the ping
#define NUDGE_GAIN_DURATION 250  //How long of invalid ping until we "nudge" gain up
#define INVALID_PING_DURATION 5000 //How long of invalid ping until gain is invalid
#define NUDGE_VALUE 0.1 //How much to nudge the gain up after the NUDGE_GAIN_DURATION

//Initializing constants for the peak detector
#define Vref 2.5             //ADC reference
#define resolution 65535     //2^16 -1
#define PINGVALIDSTART 200   //offset for the start of the ping (microseconds)
#define PINGVALIDEND 1500    //offset for the end of the ping (microseconds)
//A valid ping is 2*PEAK_NOISE
#define PEAK_NOISE 0.6
//Power rail noise is 40mV

//Constants for the filter
#define CLK0 0
#define CLK1 1
#define CLK2 2

//Watchdog timeout in ms
#define WATCHDOG_TIMEOUT 2000

#endif
