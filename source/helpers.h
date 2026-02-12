#ifndef HELPERS_H
#define HELPERS_H

#include "MicroBit.h"

/// Basic Operations
float round(float value, int step);
void floatToChar(float value, char* buffer, int step);

/// Printing
void printFloat(float value, int step = 2);
void moveCursorUp(int lines);
void moveCursorDown(int lines);

#endif // HELPERS_H
