#include "MicroBit.h"
#include "LIS3MDLSensor.h"
#include "LSM6DS3Sensor.h"

#include <cmath>
#include <cstdint>

// Microbit
MicroBit uBit;

// IMU
LSM6DS3Sensor accgyro(&uBit.i2c, 0x6A);
LIS3MDLSensor mag(&uBit.i2c, 0x1E);

// Internal
bool startStream = false;

// Helper Functions
float round(float value, int step) {
    float multiplier = 1;
    for (int i = 0; i < step; i++) multiplier *= 10;
    return (int)(value * multiplier + 0.5f) / multiplier;
}

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

void printFloat(float value, int step) {
    char buffer[12];
    floatToChar(value, buffer, step);
    uBit.serial.printf("%s", buffer);
}

void moveCursorUp(int lines) {
    for (int i = 0; i < lines; i++) {
        uBit.serial.printf("\x1b[A");   // Move cursor up
        uBit.serial.printf("\r\x1b[2K"); // Return to start of line and clear it
    }
}

// IMU
void initIMU() {
    // Gyroscope and Accelerometer
    if (accgyro.begin() != 0) {
        uBit.serial.printf("Accelerometer / Gyroscope initialization failed\r\n");
        return;
    }
    accgyro.Enable_X();
    accgyro.Enable_G();

    // Magnetometer
    if (mag.begin() != 0) {
        uBit.serial.printf("Magnetometer initialization failed\r\n");
        return;
    }
    mag.Enable_G();
}

void printAxes(int32_t* pData) {
    uBit.serial.printf("[");
    for (int i = 0; i < 3; i++) {
        uBit.serial.printf("%d", pData[i]);
        uBit.serial.printf(i < 2 ? ", " : "]");
    }
}

void printAcc() {
    int32_t axes[3];
    if (accgyro.Get_X_Axes(axes) != LSM6DS3_STATUS_OK) {
        uBit.serial.printf("Acc read failed\r\n");
        return;
    }
    uBit.serial.printf("Acc (g): ");
    printAxes(axes);
    uBit.serial.printf("\r\n");
}

void printGyro() {
    int32_t axes[3];
    if (accgyro.Get_G_Axes(axes) != LSM6DS3_STATUS_OK) {
        uBit.serial.printf("Gyro read failed\r\n");
        return;
    }
    uBit.serial.printf("Gyro (dps): ");
    printAxes(axes);
    uBit.serial.printf("\r\n");
}

void printMag() {
    int32_t axes[3];
    if (mag.Get_G_Axes(axes) != LSM6DS3_STATUS_OK) {
        uBit.serial.printf("Mag read failed\r\n");
        return;
    }
    uBit.serial.printf("Mag (uT): ");
    printAxes(axes);
    uBit.serial.printf("\r\n");
}

void printIMU() {
    printAcc();
    printGyro();
    printMag();
}

// Button A event handler
static void onButtonA(MicroBitEvent) {
    // calibrate();
    startStream = true;
}

int main() {
    uBit.init();

    // Test out the IMU Components
    initIMU();

    // Listen for the A Button
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    
    // Print Statement
    uBit.serial.printf("IMU initialized. Press A to start streaming.");

    while (true) {
        if (startStream) {
            moveCursorUp(5);
            printIMU();
            uBit.sleep(100);
        } else {
            uBit.sleep(100);
        }
    }
}