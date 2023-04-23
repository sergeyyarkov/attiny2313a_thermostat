SRC_FILE					= firmware
MCU								= t2313
PROGRAMMER				= usbtiny
BUILD_DIR					= build

all:	build_dir
			avrasm2 -fI -W+ie $(SRC_FILE).asm -o ./$(BUILD_DIR)/$(SRC_FILE).hex -l ./$(BUILD_DIR)/$(SRC_FILE).lss

flash:
			avrdude -c $(PROGRAMMER) -p $(MCU) -U flash:w:./$(BUILD_DIR)/$(SRC_FILE).hex:i

clean:	
			rm -f $(BUILD_DIR)/*

build_dir:
			mkdir -p ${BUILD_DIR}