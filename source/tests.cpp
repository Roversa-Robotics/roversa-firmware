#include "MicroBit.h"
#include "helpers.h"
#include "tests.h"

// Variables
extern MicroBit uBit;

// I2C
void testI2CDevices() { // Serial Prints which I2C Addressed are valid and are writing and reading
    uBit.serial.printf("Testing all I2C addresses...\r\n");

    for (uint8_t addr = 0x08; addr <= 0x7F; addr++) {
        uint8_t reg = 0x00;
        uint8_t data[1] = {0};

        int writeResult = uBit.i2c.write(addr << 1, &reg, 1, true);
        int readResult = uBit.i2c.read(addr << 1, data, 1);

        if (writeResult == 0 || readResult == 0) {
            uBit.serial.printf("Valid I2C device at address %d (0x%x)\r\n", addr, addr);
            uBit.serial.printf("Write result: %d, Read result: %d, Data: %d\r\n", writeResult, readResult, data[0]);
        }
    }
}

void testVector3(Vector3 v1, Vector3 v2, int step) {
    // Initial Print
    uBit.serial.printf("Testing Vector3 Methods...\r\n");

    uBit.serial.printf("v1: ");
    printVector3(v1, step);
    uBit.serial.printf(", v2: ");
    printVector3(v2, step);
    uBit.serial.printf("\r\n");
    
    // Addition
    uBit.serial.printf("\r\nv1 + v2: ");
    printVector3(v1 + v2, step);

    // Subtraction
    uBit.serial.printf("\r\nv1 - v2: ");
    printVector3(v1 - v2, step);

    // Multiplication
    uBit.serial.printf("\r\nv1 * 3: ");
    printVector3(v1 * 3, step);

    // Division
    uBit.serial.printf("\r\nv1 / 3: ");
    printVector3(v1 / 3, step);
    
    // Length
    uBit.serial.printf("\r\nv1.length(): ");
    printFloat(v1.length(), step);

    // Normalization
    uBit.serial.printf("\r\nv1.normalize(): ");
    printVector3(v1.normalize(), step);
    
}