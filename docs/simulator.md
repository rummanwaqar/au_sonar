# Sonar Simulator

Sonar simulator generates synthetic ping data based on the relative positions of the robot and the pinger. 
This is a direct replacement for the beaglebone hardware. Simulated data is sent over ZMQ similar to the actual hardware.

## Run

`roslaunch au_sonar sonar_sim`

This launch file launches the sonar simulator as well as the sonar processing node.

### Params:

* `sampling_freq`: ADC sampling freq (Hz)
* `pinger_freq`: Target pinger frequency (Hz)
* `noise_stdev`: Noise to add to heading (degrees)
* `x`: pinger x position (m)
* `y`: pinger y position (m)
* `rate`: ping interval (Hz)

#### Important notes
* Make sure that pinger coods are in NED.
* Make sure that state_aggregator is runnning. Simulator uses pose to simulate pinger data.
* Simulator does not add any physical objects to the simulator. They have to be added manually in the model.
 