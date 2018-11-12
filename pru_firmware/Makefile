RM := $(RM)
PASM := pasm -DBUILD_WITH_PASM=1

INCLUDE_DIR := ../include

.PHONY: all clean install

TARGETS := pru0-clock.bin pru1-read-data.bin

all: $(TARGETS)

clean:
		$(RM) $(TARGETS)
		
%.bin: %.p
		$(PASM) -I$(INCLUDE_DIR) -b $< $(basename $^)