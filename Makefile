ARDUINO_CC ?= arduino-cli
ARDUINO_BOARD ?= arduino:avr:uno
ARDUINO_DEVICE ?= /dev/ttyACM0
ARDUINO_BIN_LOC = bin/
ARDUINO_BIN_FILES ?= avr_dmx

.PHONY: build

build:
	@mkdir -p $(ARDUINO_BIN_LOC)
	$(ARDUINO_CC) compile --fqbn $(ARDUINO_BOARD) avr_dmx -o $(ARDUINO_BIN_LOC)$(ARDUINO_BIN_FILES)

upload:
	$(ARDUINO_CC) upload -p $(ARDUINO_DEVICE) --fqbn $(ARDUINO_BOARD) -i $(ARDUINO_BIN_LOC)$(ARDUINO_BIN_FILES) avr_dmx
