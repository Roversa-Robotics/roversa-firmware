#include "helpers.h"

//// MicroBit
extern MicroBit uBit;

//// Basic Operations
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

//// Printing
// Printing a String: uBit.serial.printf("")
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

void moveCursorDown(int lines) {
    for (int i = 0; i < lines; i++) {
        uBit.serial.printf("\r\n");   // Move cursor down
    }
}
