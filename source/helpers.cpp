#include "MicroBit.h"
#include "helpers.h"

// Variables
extern MicroBit uBit;

// Vector3 Implementation
Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3 Vector3::operator+(const Vector3& other) const {
	return Vector3(x + other.x, y + other.y, z + other.z);
}

Vector3 Vector3::operator-(const Vector3& other) const {
	return Vector3(x - other.x, y - other.y, z - other.z);
}

Vector3 Vector3::operator*(float scalar) const {
	return Vector3(x * scalar, y * scalar, z * scalar);
}

Vector3 Vector3::operator/(float scalar) const {
	return (scalar != 0) ? Vector3(x / scalar, y / scalar, z / scalar) : Vector3();
}

float Vector3::length() const {
	return std::sqrt(x * x + y * y + z * z);
}

Vector3 Vector3::normalize() const {
	float len = length();
	return (len != 0) ? *this / len : Vector3();
}

void Vector3::Vector3ToChar(char* buffer, int step) const {
	char xBuffer[12], yBuffer[12], zBuffer[12];
	floatToChar(x, xBuffer, step);
	floatToChar(y, yBuffer, step);
	floatToChar(z, zBuffer, step);

	int index = 0;
	buffer[index++] = '[';

	for (int i = 0; xBuffer[i] != '\0'; i++)
		buffer[index++] = xBuffer[i];

	buffer[index++] = ',';
	buffer[index++] = ' ';

	for (int i = 0; yBuffer[i] != '\0'; i++)
		buffer[index++] = yBuffer[i];

	buffer[index++] = ',';
	buffer[index++] = ' ';

	for (int i = 0; zBuffer[i] != '\0'; i++)
		buffer[index++] = zBuffer[i];

	buffer[index++] = ']';
	buffer[index] = '\0';
}

// Serial Printing
void printVector3(const Vector3& vec, int step) {
	char buffer[50]; // Allocate buffer for formatted output
	vec.Vector3ToChar(buffer, step);
	uBit.serial.printf("%s", buffer);
}

void printFloat(float value, int step) {
    char buffer[12]; // Allocate buffer for formatted output
    floatToChar(value, buffer, step);
    uBit.serial.printf("%s", buffer);
}

void moveCursorUp(int lines) {
	for (int i = 0; i < lines; i++) {
		uBit.serial.printf("\x1b[A");
	}
}

// Data Conversion
float round(float value, int step) {
	float multiplier = 1;
	for (int i = 0; i < step; i++) {
		multiplier *= 10;
	}
	return (int)(value * multiplier + 0.5) / multiplier;
}

int16_t roundToInt(float value) {
	return (int16_t)(value + (value >= 0 ? 0.5f : -0.5f));
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
	do {
		temp /= 10;
		digits++;
	} while (temp > 0);

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
