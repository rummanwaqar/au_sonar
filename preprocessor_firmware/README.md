# au_sonar_firmware
A repository for Teensy 3.2 firmware on sonar preprocessor V3 board

# Functions

## Variable Gain Amplifier (VGA) Function Documentation 

This is a function for setting the gain of the AD8336 variable gain amplifier (VGA)
using the digitial to analog converter of a teensy 3.2 to control 
the GPOS pin on the VGA with the pre-amplifier set to 26dB and the GNEG
at a reference voltage of 0.9V.

To initialise the  ```gain_control()``` function, analogWriteResolution is called in main.cpp to establish 12 bits of DAC resolution. 

```c++
}
  analogWriteResolution(12); // setup DAC for Amplifier
}
```

The ```gain_control()``` function accepts a gain value in dB as its one and only argument. The function is
built according to the composite gain formula found on page 21 of the datasheet: 

<a href="https://ibb.co/iQ3eLd"><img src="https://image.ibb.co/nPEVRJ/Capture.png" alt="Capture" border="0"></a>

Where:

Composite gain is the total gain of the system including the preamplifier

GPRA is the gain of the preamplifier (pre-set to 26dB in function definition)

Vgain is the difference in voltage applied to the attenuator to control gain. Vgain=GPOS-GNEG

As seen in the graph below, a Vgain of at least -600mV is required to reach 0dB (Note: 20x = 20V/V =~26dB). Because the teensy 
is unable to output negative voltages, the GNEG pin is set at a reference voltage of 0.9V (+0.3V from 
0.6V from minimum for stability). 

<a href="https://ibb.co/iexrWJ"><img src="https://image.ibb.co/eBcrWJ/image.png" alt="image" border="0"></a>

The formula in the datasheet is rearranged to calculate the necessary voltage to send to the GPOS pin
in order to achieve the required desired dB gain level that is inputed by the user as an argument to the
function. Therefore, to call the function, just pass it the desired voltage gain in dB and it will output the necessary 
voltage to achieve the gain. 

The minimum voltage gain is set to 0dB and the maximum allowable voltage gain is 60dB. The function will display an error in the serial port if these extremes are violated by the user. 

# PeakDetector Class
There are three parameters to adjust to ensure the proper function of the peak detector:

-PEAK_NOISE is in the pinout.h file and is set to exclude any noise of the signal. A valid signal is >= 2*PEAK_NOISE.

-PINGVALIDSTART is defined in the peak_detector.h file and is set in microseconds as the time from the first valid pinglevel to when the ping finishes ramping up.

-PINGVALIDEND is defined in the peak_detector.h file and is set in microseconds as the time from when the ping is stable to when it starts to ramp down.

### PeakDetector()

Initialization for the peak detector funcions. It sets the clear pin for the BJT and GPIO_3 (pin 11) for monitoring to output modes. Then sets up the ADC by setting the reference voltage to AREF, averaging to 16, sampling speed to medium, conversion speed to high, and the resolution to 16bit.

### int getPingStatus(float &peaklevel)

Takes the input from the peak detector and converts it to a voltage using: peaklevel = (Vref*peakinput)/(resolution) and checks if it's valid. A valid ping is 2*PEAK_NOISE or greater. 
The function uses elaspedmicros to remove the ramps and the start and end of the ping to clean up the data.

Returns 1 if there is a valid ping                                                       
Returns 2 if the ping is valid but ramping down                                          
Returns 3 if the ping is valid but still ramping up                                     
Returns 0 if the ping is invalid

### resetDetector()

Sets the PEAK_CLEAR_PIN high (turns the BJT off) then starts an intervaltimer to call the detectorlow function.

### detectorlow()

Sets the PEAK_CLEAR_PIN low (turns the BJT on) then ends the intervaltimer which would otherwise keep calling detectorlow every 25microseconds.

# Communication Protocol

See ![docs/CommInterface.md](docs/CommInterface.md) for information on the communication interface along with the communication protocol. 
	
  

