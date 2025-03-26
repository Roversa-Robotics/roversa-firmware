#include "MicroBit.h"
// #include "LSM6DS3Sensor.h"
#include "LIS3MDLSensor.h"

#include <cmath>
#include <cstdint>

MicroBit uBit;
// LSM6DS3Sensor imu(&uBit.i2c, 0x6A);
LIS3MDLSensor mag(&uBit.i2c, 0x1E);
bool startMagStream = false;

// Round a float to a specified number of decimal places
float round(float value, int step) {
    float multiplier = 1;
    for (int i = 0; i < step; i++) multiplier *= 10;
    return (int)(value * multiplier + 0.5f) / multiplier;
}

// Convert float to character array for serial printing
void floatToChar(float value, char* buffer, int step) {
    value = round(value, step);
    int integerPart = (int)value;
    int decimalPart = (int)((value - integerPart) * 100);

    int bufferIndex = 0;
    if (value < 0) {
        buffer[bufferIndex++] = '-';
        integerPart = -integerPart;
        decimalPart = -decimalPart;
    }

    int temp = integerPart, digits = 0;
    do { temp /= 10; digits++; } while (temp > 0);

    for (int i = digits - 1; i >= 0; i--) {
        buffer[bufferIndex + i] = '0' + (integerPart % 10);
        integerPart /= 10;
    }
    bufferIndex += digits;
    buffer[bufferIndex++] = '.';

    for (int i = 1; i >= 0; i--) {
        buffer[bufferIndex + i] = '0' + (decimalPart % 10);
        decimalPart /= 10;
    }
    bufferIndex += 2;
    buffer[bufferIndex] = '\0';
}

// Helper to print formatted float values
void printFloat(float value, int step) {
    char buffer[12];
    floatToChar(value, buffer, step);
    uBit.serial.printf("%s", buffer);
}

// Moves the terminal cursor up by the given number of lines
void moveCursorUp(int lines) {
    for (int i = 0; i < lines; i++) {
        uBit.serial.printf("\x1b[A");
    }
}

// Print magnetometer values with optional precision control
void printMag(int step = 3) {
    int32_t axes[3];
    if (mag.GetAxes(axes) != LIS3MDL_STATUS_OK) {
        uBit.serial.printf("Mag read failed\r\n");
        return;
    }
    uBit.serial.printf("Mag (uT): [");
    for (int i = 0; i < 3; i++) {
        printFloat((float)axes[i], step);
        uBit.serial.printf(i < 2 ? ", " : "");
    }
    uBit.serial.printf("]\r\n");
}

// Initializes the magnetometer
void testMag() {
    if (mag.begin() != 0) {
        uBit.serial.printf("Magnetometer initialization failed\r\n");
        return;
    }

    mag.Enable();
    mag.SetFS(4.0f);
    mag.SetODR(80.0f);

    uBit.serial.printf("Magnetometer initialized. Press A to start streaming.\r\n\r\n\r\n");
}

// Button A event handler
static void onButtonA(MicroBitEvent) {
    // calibrate();
    startMagStream = true;
}

int main() {
    uBit.init();
    testMag();

    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);

    while (true) {
        if (startMagStream) {
            moveCursorUp(3);
            printMag();
            uBit.sleep(10);
        } else {
            uBit.sleep(100);
        }
    }
}