#!/usr/bin/env bash

# path to source directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# check if build folder exists
if ! [ -d "$DIR/build" ]; then
    echo "build/ not found. Recompile and try again."
    exit 1
fi

# run i/o configuration
echo "Setting up I/O"
bash $DIR/setup_pins.sh

# run daq program
sudo $DIR/build/pru_read $DIR/pru_firmware/pru0-clock.bin $DIR/pru_firmware/pru1-read-data.bin