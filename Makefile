CFLAGS += --std=c++11 -O2 -Wall

CC := g++
RM := $(RM)
PASM := pasm -DBUILD_WITH_PASM=1

SRC_DIR := src
FIRMWARE_DIR := firmware
INCLUDE_DIR := ./include

.PHONY: all clean install

TARGETS := pru_read $(FIRMWARE_DIR)/pru0-clock.bin $(FIRMWARE_DIR)/pru1-read-data.bin

all: $(TARGETS)

clean:
		$(RM) $(TARGETS)
		
%.bin: %.p
		$(PASM) -I$(INCLUDE_DIR) -b $< $(basename $^)

pru_read: $(SRC_DIR)/pru_read.cpp
		$(CC) $(CFLAGS) -o $@ $^ -l prussdrv -I$(INCLUDE_DIR)