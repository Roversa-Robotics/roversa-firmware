#ifndef HELPERS_H
#define HELPERS_H

#include <cmath>

// Vector3
class Vector3 {
public:
	float x, y, z;

	// Constructor
	Vector3(float x = 0, float y = 0, float z = 0);

	// Addition
	Vector3 operator+(const Vector3& other) const;

	// Subtraction
	Vector3 operator-(const Vector3& other) const;

	// Multiplication by scalar
	Vector3 operator*(float scalar) const;

	// Division by scalar
	Vector3 operator/(float scalar) const;

	// Get Length of Vector
	float length() const;

	// Normalize the Vector
	Vector3 normalize() const;

    // Convert the Vector
    void Vector3ToChar(char* buffer, int step) const;
};

// Serial Print
void moveCursorUp(int lines);
void printVector3(const Vector3& vec, int step);
void printFloat(float value, int step);


// Data Conversion
float round(float value, int step);
int16_t roundToInt(float value);
void floatToChar(float value, char* buffer, int step);


#endif // HELPERS_H
