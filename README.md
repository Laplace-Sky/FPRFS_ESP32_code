# FPRFS_ESP32_code
This repository contains all codes required for the FPRFS related hardware.

With the hardware, open the “ESP32_code.ino” in Arduino IDE and load the program to the ESP32 board, which connects to an FPRFS or a series of cascaded FPRFSs. Set the local IP address for the ESP32 and the number of boards cascaded for testing. All the rest of the functions introduced in our work can be achieved and controlled in the GUI. 

For RF -meter-based self-adapting tests, load the “SPI_read_from_RFMETER_final.ino” to the ESP32 board communicating with the RF meter.
