SKETCH ?= drone
FQBN   ?= esp32:esp32:esp32s3:CDCOnBoot=cdc
PORT   ?= /dev/ttyACM0
BAUD   ?= 115200

.PHONY: build upload flash monitor

build:
	arduino-cli compile --fqbn $(FQBN) $(SKETCH)

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) $(SKETCH)

# build + upload in one go
flash: build upload

monitor:
	arduino-cli monitor -p $(PORT) -c baudrate=$(BAUD)
