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

# make log file
LOG_FILE="sonar_$(date +"%F-%T").txt"

# run daq program
$DIR/build/sonar_daq --pru0 $DIR/build/pru0-clock.bin --pru1 $DIR/build/pru1-read-data.bin --log $LOG_FILE --cmd_server tcp://127.0.0.1:6868 --data_server tcp://127.0.0.1:6969
