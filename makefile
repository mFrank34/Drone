.PHONY: build upload monitor clean

build:
	pio run

upload:
	pio run -t upload

flash: upload

monitor:
	pio device monitor

clean:
	pio run -t clean
