# Passive Sonar Subsystem

## Building the code
Make sure all dependiencies are installed. Check [Beaglebone Setup Guide](docs/bbb-install.md).
```
mkdir build && cd $_
cmake ..
make
```
*Please note that the Beaglebone specific code only compiles on a Beaglebone. It will exit cleanly without compiling on any other system.*

## Running the code
Each command is run from the root in this example.
* Setup IO `./scripts/setup_pins.sh`
* Start the data acqusition server `sudo ./build/pru_read ./pru_firmware/pru0-clock.bin ./pru_firmware/pru1-read-data.bin`
* Run the python processing code `python ./scripts/sonar.py`

## Pinouts

## Documentation
* [Beaglebone Setup Guide](docs/bbb-install.md)
* [Beaglebone Internal Pinout](docs/pinout.md)
