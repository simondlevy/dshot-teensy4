SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = teensy:avr:teensy40

build: $(SKETCH).ino
	arduino-cli compile --libraries \
		$(HOME)/Documents/Arduino/libraries --libraries $(ROOT)/.. \
		--fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

listen:
	miniterm.py $(PORT) --exit-char 3

edit:
	vim $(SKETCH).ino

