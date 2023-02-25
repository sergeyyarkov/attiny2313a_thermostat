SRC_FILE = firmware
MCU = t2313
PROGRAMMER = usbtiny

build:
	avrasm2 -fI -W+ie $(SRC_FILE).asm -l $(SRC_FILE).lss

flash:
	avrdude -c $(PROGRAMMER) -p $(MCU) -U flash:w:$(SRC_FILE).hex:i