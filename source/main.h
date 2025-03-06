#ifndef HELPERS_H
#define HELPERS_H

// Vector3
class Vector3 {
public:
	float x, y, z;

	// Constructor
	Vector3(float x = 0, float y = 0, float z = 0);

	// Algebra
	Vector3 operator+(const Vector3& other) const;
	Vector3 operator-(const Vector3& other) const;
	Vector3 operator*(float scalar) const;
	Vector3 operator/(float scalar) const;

	float length() const;
	Vector3 normalize() const;

	// Annoying Math
	float dot(const Vector3& other) const;
	Vector3 cross(const Vector3& other) const;

    // Misc
    void Vector3ToChar(char* buffer, int step) const;
};


#endif // HELPERS_H
