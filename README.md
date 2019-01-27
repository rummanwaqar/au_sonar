# Passive Sonar Triangulation System

The sonar system is divided into two parts:
* Sonar DAQ system (on Beaglebone)
* Sonar processing system (on TX2)

The systems communicate using ZMQ; Sonar DAQ runs a command server to forward commands to preprocessor. It also publishes all serial data over a ZMQ publisher.

## Building the code

#### Beaglebone Specific Code
Make sure all dependencies are installed. Check [Beaglebone Setup Guide](docs/bbb-install.md).

Unit tests are enabled by setting the `TESTS` flag to 1. They are done through the [googletest](https://github.com/google/googletest) framework.

Debugging information can be enabled to allow for debugging through GDB by setting the `DEBUG` flag to 1.
```
cd beaglebone
mkdir build && cd $_
cmake [-DTESTS=0/1] [-DDEBUG=0/1] ..
make
```

#### ROS Based code
Compile with `catkin_make`

## Running the code

#### Beaglebone Data Acqusition Server
Run `beaglebone/run.sh`

#### ROS node
Run `roslaunch au_sonar sonar_node.launch freq:=<target freq in Khz>`

## Documentation
* [Sonar Simulator](docs/simulator.md)
* [Wiring Guide](docs/wiring.md)
* [Preprocessor documentation](preprocessor_firmware/README.md)
* [Preprocessor communication interface](preprocessor_firmware/docs/CommInterface.md)
* [Beaglebone Setup Guide](docs/bbb-install.md)
* [Beaglebone Internal Pinout](docs/pinout.md)
