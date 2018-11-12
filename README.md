# Passive Sonar Triangulation System

## Building the code

#### Beaglebone Specific Code
Make sure all dependiencies are installed. Check [Beaglebone Setup Guide](docs/bbb-install.md).
```
cd beaglebone
mkdir build && cd $_
cmake ..
make
```
*Please note that the Beaglebone specific code only compiles on a Beaglebone. It will exit cleanly without compiling on any other system.*

#### ROS Based code
Compile with `catkin_make`

## Running the code

#### Beaglebone Data Acqusition Server
Run `beaglebone/run.sh` 

#### ROS node


## Documentation
* [Sonar Simulator](docs/simulator.md)
* [Wiring Guide](docs/wiring.md)
* [Preprocessor documentation](preprocessor_firmware/README.md)
* [Preprocessor communication interface](preprocessor_firmware/docs/CommInterface.md)
* [Beaglebone Setup Guide](docs/bbb-install.md)
* [Beaglebone Internal Pinout](docs/pinout.md)