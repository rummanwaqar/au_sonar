# Beaglebone Pinout

This doc describes the GPIO pins and corresponding PRU registers used to interface with the board.

## System diagram
![System diagram](https://github.com/google/prudaq/wiki/conceptual_schematic.png)

## PRU0
| PRU register        	| GPIO pin 	| Mode 	| Description              	|
|---------------------	|----------	|------	|--------------------------	|
| pr1_pru0_pru_r30_0  	| P9_31    	| O    	| GPIO generated ADC clock 	|
| pr1_pru0_pru_r30_1  	| P9_29    	| O    	| INPUT0A                  	|
| pr1_pru0_pru_r30_2  	| P9_30    	| O    	| INPUT0B                  	|
| pr1_pru0_pru_r30_3  	| P9_28    	| O    	| INPUT1A                  	|
| pr1_pru1_pru_r31_5  	| P9_27    	| O    	| INPUT1B                  	|
| pr1_pru0_pru_r3x_14 	| P8_16    	|      	| Unused                   	|
| pr1_pru0_pru_r3x_15 	| P8_15    	|      	| Unused                   	|
| pr1_pru0_pru_r31_16 	| P9_24    	| I    	| Unused                   	|

## PRU1

| PRU pin             	| GPIO pin 	| Mode 	| Description                 	|
|---------------------	|----------	|------	|-----------------------------	|
| pr1_pru1_pru_r31_0  	| P8_45    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_1  	| P8_46    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_2  	| P8_43    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_3  	| P8_44    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_4  	| P8_41    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_5  	| P8_42    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_6  	| P8_39    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_7  	| P8_40    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_8  	| P8_27    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_9  	| P8_29    	| I    	| ADC data out                	|
| pr1_pru1_pru_r31_10 	| P8_28    	| I    	| INPUT0B re-routed to PRU1   	|
| pr1_pru1_pru_r31_11 	| P8_30    	| I    	| ADC clock re-routed to PRU1 	|
| pr1_pru1_pru_r31_12 	| P8_21    	|      	| Unused                      	|
| pr1_pru1_pru_r31_13 	| P8_20    	|      	| Unused                      	|
| pr1_pru1_pru_r31_16 	| P9_26    	| I    	| INPUT0A re-routed to PRU1    	|
