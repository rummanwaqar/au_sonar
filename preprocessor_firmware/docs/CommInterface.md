# CommInterface Class 

## Protocol

All transmissions shall begin with a $ when the device is not a "debug" mode
All transmissions shall end with a newline character ( '\n' )

Format is as follows:

  "$[command] [variable] [argument(s)]'\n'"

For example if you wanted to set the gain to 10.0 dB:

  "$set gain 10.0" 

  Keep in mind there should be a newline character at the end.

### Commands

Currently there are only two commands; "set" and "get"

"set" will allow you to set a variable to a certain value.

"get" will retrieve a variable for you over serial. The value will start with a '$' and end with '\n'

For example a "get" response may look like:

  "$get pGain"

  "$0.1"

### Variables

Currently there are 15 variables that can be accessed:

  "desiredPeak" which is the target peak for the gain controller
  

  "pGain" the proportional gain for the gain controller

  "iGain" the integral gain for the gain controller

  "iSaturation" the integral error saturation for the gain controller
  "iSaturation takes TWO arguments; min then max

  "floorGainDur" the duration in ms to set gain to 0dB after detected ping

  "nudgeGainDur" the duration after consecutive invalid pings to nudge the gain up a bit

  "invalidPingDur" the duration after consecutive invalid pings to print debug info

  "nudgeGainValue" the amount to nudge the gain

  "validPingStart" the duration in us at the start of ping to not feed into gain controller

  "validPingEnd" the duration of the ping after it gets fed into the gain controller, afterwards it gets floored

  "gain" the gain of the controller in dB

  "holdGain" a boolean than prevents the gain controller from modifying the gain
  
  "peakLevel" the peak level of the peak detector in V
  
  "pingStatus" The status of ping. 0=invalid ping, 1=valid ping, 2=end of ping, 3=start of ping
  
  "centerFreq" The center frequency of the bandpass filter. 

  "debug" a boolean that indictes if debug mode is enabled

  "adcAveraging" a integer that sets how many samples the adc averages over

  "validMean" a float. Its the maximum distance from the desiredPeak for the calibration to consider the mean of the signal valid.

  "validVariance" a float. The maximum distance from zero that the variance can be for the calibration to consider the variance valid.

### Arguments

Arguments are used in the case of a "set" command

Arguments may be floats or negative numbers

"iSaturation" variable requires two arguments example:

  "$set iSaturation -10.00 5.15"

Apon sending this data; the comm interface will echo what was sent if it started with '$' and ended with '/n'

### Ping Feedback Info

Upon initialization, the teensy will print "$Teensy Initialized", with a new line character at the end. This can be used to see if the teensy previously crashed.

After a ping, the gain controller will send some feed back informatio;

  "cal" a boolean: if the gaincontroller thinks the ping has been calibrated.
  
  "gain" a float with 2 decimal precision. The current gain of the amplifier in dB.

  "peakLevel" a float with 2 decimal precision. The current peak level of the peak detector

  "avgPkLv" a float with 2 decimal precision. The average peak level of the last ping.

  "variance" a float with 5 decimal precision. The variance of the peak level of the last ping.

  "samples" a integer. The amount of peak detector samples, and thus the amount times gain control looped during the ping.



Example: "$ping cal=0 gain=40.45 peakLevel=1.23 avgPkLv=1.28 variance=0.00567 samples=19"

Note: There's a new line character at the end of the string
