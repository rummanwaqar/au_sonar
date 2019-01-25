RM := $(RM)
PASM = pasm
BUILD_CMD := $(PASM) -DBUILD_WITH_PASM=1
BUILD_DIR = ../build

INCLUDE_DIR := ../include

.PHONY: all clean install

TARGETS := pru0-clock.bin pru1-read-data.bin

all: $(TARGETS)

clean:
		$(RM) $(TARGETS)

%.bin: %.p
		$(BUILD_CMD) -I$(INCLUDE_DIR) -b $< $(BUILD_DIR)/$(basename $^)
