# Sonar Simulator

Sonar simulator is a 2D simulator that returns relative heading to the acoustic pinger.

### Important notes
* Make sure that pinger coods are in NED.
* Make sure that state_aggregator is runnning. Simulator uses pose to simulate pinger data.

### Params:

* `a_x`: pingerA x cood (m)
* `a_y`: pingerA y cood (m)
* `b_x`: pingerB x cood (m)
* `b_y`: pingerB y cood (m)
* `processing_time`: simulated sonar processing time (ms)
* `pose_topic`: topic to get auri simulated pose to calculate heading
* `noise`: +/- noise in output value of simulator (in degrees)

#### Action:
Output headings are in degrees (or NaN). 
```
Goal: empty
Feedback: (string) calibration/calculating
Result: (float32) rel_heading, abs_heading
```

#### Switching Pingers
Pingers can be switched by publishing to `/sonar/enable`.

* Disable: Set value to 0
* PingerA: Set value to 1
* PingerB: Set value to 2



**Sonar simulator is included in the simulator.launch** and can be configured by setting the following args:

* `<arg name="sonar_on" value="True" />` Turns on the sonar simulator
* `<arg name="pingerA_x" value="23.9" />` Sets the x-cood for the pingerA
* `<arg name="pingerA_y" value="23.6"/>` Sets the y-cood for the pingerA
* `<arg name="pingerB_x" value="37.3" />` Sets the x-cood for the pingerB
* `<arg name="pingerB_y" value="23.9"/>` Sets the y-cood for the pingerB

Example launch file with sonar enabled:

```xml
<launch>
    <include file="$(find underwater_vehicle_dynamics)/launch/simulator.launch">
        <arg name="vehicle" value="auri"/>
        <arg name="scene" value="$(find uwsim)/data/scenes/robosub18_semifinal.xml" />

        <arg name="sonar_on" value="True" />
        <arg name="pingerA_x" value="23.9" />
        <arg name="pingerA_y" value="23.6"/>
        <arg name="pingerB_x" value="37.3" />
        <arg name="pingerB_y" value="23.9"/>
    </include>
</launch>
```

Note: This does not add any physical objects to the simulator. They have to be added manually in the model.