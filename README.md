# Passive Sonar Triangulation System

## Building the code

#### Beaglebone Specific Code
Make sure all dependiencies are installed. Check [Beaglebone Setup Guide](docs/bbb-install.md).
```
mkdir build && cd $_
cmake ..
make
```
*Please note that the Beaglebone specific code only compiles on a Beaglebone. It will exit cleanly without compiling on any other system.*

#### ROS Based code
Compile with `catkin_make`

## Running the code

#### On beaglebone
Each command is run from the project root in this example:
* Setup IO `./scripts/setup_pins.sh`
* Start the data acqusition server `sudo ./build/pru_read ./pru_firmware/pru0-clock.bin ./pru_firmware/pru1-read-data.bin`
* [Optional] Saving pings to cvs `python ./scripts/save_ping.py path [--angle angle]`

#### On ROS machine


## Documentation
* [Sonar Simulator](docs/simulator.md)
* [Wiring Guide](docs/wiring.md)
* [Preprocessor documentation](preprocessor_firmware/README.md)
* [Preprocessor communication interface](preprocessor_firmware/docs/CommInterface.md)
* [Beaglebone Setup Guide](docs/bbb-install.md)
* [Beaglebone Internal Pinout](docs/pinout.md)