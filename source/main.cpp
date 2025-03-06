#include "MicroBit.h"
#include "servoRoversa.h"
#include "main.h"

#include <cmath>
#include <cstdint>

// Microbit
MicroBit uBit;

//////////////////////////////////////////////////////////////////////////////
// Vector3
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

float Vector3::dot(const Vector3& other) const {
	return x*other.x + y*other.y + z*other.z;
}

Vector3 Vector3::cross(const Vector3& other) const {
	return Vector3(y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x);
}

// IMU
struct IMU {
    Vector3 accel;
    Vector3 gyro;
    Vector3 mag;
};

// State
struct State {
    Vector3 position;
    Vector3 velocity;
    Vector3 orientation;
    Vector3 bias;
};

//////////////////////////////////////////////////////////////////////////////
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

void printIMU(IMU imu, int step) {
    
    // Accelerometer
    uBit.serial.printf("Accel: ");
    printVector3(imu.accel, step);

    // Gyroscope
    uBit.serial.printf("\r\nGyro: ");
    printVector3(imu.gyro, step);

    // Magnetometer
    uBit.serial.printf("\r\nMag: ");
    printVector3(imu.mag, step);
}

void printState(State state, int step) {
    
    // Position
    uBit.serial.printf("Position: ");
    printVector3(state.position, step);

    // Velocity
    uBit.serial.printf("\r\nVelocity: ");
    printVector3(state.velocity, step);

    // Orientation
    uBit.serial.printf("\r\nOrientation: ");
    printVector3(state.orientation, step);

    // Bias
    uBit.serial.printf("\r\nBias: ");
    printVector3(state.bias, step);
}

void moveCursorUp(int lines) {
	for (int i = 0; i < lines; i++) {
        uBit.serial.printf("\r                                                ");
		uBit.serial.printf("\x1b[A");
	}
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

//////////////////////////////////////////////////////////////////////////////
// I2C Addresses
#define LSM6DS3TR_C_ADDR 0x6A // Accelerometer & Gyroscope
#define LIS3MDL_ADDR 0x1E // Magnetometer

// Register addresses
#define CTRL1_XL 0x10 // Accelerometer control
#define CTRL2_G 0x11 // Gyroscope control
#define OUTX_L_A 0x28 // Accelerometer X low byte
#define OUTX_L_G 0x22 // Gyroscope X low byte
#define OUTX_L_M 0x28 // Magnetometer X low byte

// Sensitivites (set after the init function is called)
float ACCEL_SENSITIVITY;
float GYRO_SENSITIVITY;
float MAG_SENSITIVITY;

// Initialization
void initIMU(uint8_t accelFS = 2, uint16_t gyroFS = 125) {
    uint8_t accelConfig = 0x00;
    uint8_t gyroConfig = 0x00;

    // Set accelerometer sensitivity
    if (accelFS == 2) {
        ACCEL_SENSITIVITY = 0.000061;
        accelConfig = 0x00;
    } else if (accelFS == 4) {
        ACCEL_SENSITIVITY = 0.000122;
        accelConfig = 0x08;
    } else if (accelFS == 8) {
        ACCEL_SENSITIVITY = 0.000244;
        accelConfig = 0x0C;
    } else if (accelFS == 16) {
        ACCEL_SENSITIVITY = 0.000488;
        accelConfig = 0x04;
    } else {
        uBit.serial.printf("Error: Invalid accelerometer sensitivity: %d\r\n", accelFS);
        return;
    }

    // Set gyroscope sensitivity
    if (gyroFS == 125) {
        GYRO_SENSITIVITY = 0.004375;
        gyroConfig = 0x02;
    } else if (gyroFS == 250) {
        GYRO_SENSITIVITY = 0.00875;
        gyroConfig = 0x00;
    } else if (gyroFS == 500) {
        GYRO_SENSITIVITY = 0.0175;
        gyroConfig = 0x08;
    } else if (gyroFS == 1000) {
        GYRO_SENSITIVITY = 0.035;
        gyroConfig = 0x0C;
    } else if (gyroFS == 2000) {
        GYRO_SENSITIVITY = 0.07;
        gyroConfig = 0x04;
    } else {
        uBit.serial.printf("Error: Invalid gyroscope sensitivity: %d\r\n", gyroFS);
        return;
    }

    // Write to I2C
    uint8_t ctrl1_xl[2] = {CTRL1_XL, (0x60 | accelConfig)};
    uint8_t ctrl2_g[2] = {CTRL2_G, (0x60 | gyroConfig)};
    uBit.i2c.write(LSM6DS3TR_C_ADDR << 1, ctrl1_xl, 2);
    uBit.i2c.write(LSM6DS3TR_C_ADDR << 1, ctrl2_g, 2);
}
void initMagnetometer(uint8_t magFS = 4) {
    uint8_t magConfig = 0x00;

    if (magFS == 4) {
        MAG_SENSITIVITY = 0.00014;
        magConfig = 0x00;
    } else if (magFS == 8) {
        MAG_SENSITIVITY = 0.00029;
        magConfig = 0x20;
    } else if (magFS == 12) {
        MAG_SENSITIVITY = 0.00043;
        magConfig = 0x40;
    } else if (magFS == 16) {
        MAG_SENSITIVITY = 0.00058;
        magConfig = 0x60;
    } else {
        uBit.serial.printf("Error: Invalid magnetometer sensitivity: %d\r\n", magFS);
        return;
    }

    // Write to I2C
    uint8_t config1[2] = {0x20, 0x70}; // Enable magnetometer, 10Hz ODR
    uint8_t config2[2] = {0x21, magConfig}; // Set full-scale
    uint8_t config3[2] = {0x22, 0x00}; // Continuous mode

    uBit.i2c.write(LIS3MDL_ADDR << 1, config1, 2);
    uBit.i2c.write(LIS3MDL_ADDR << 1, config2, 2);
    uBit.i2c.write(LIS3MDL_ADDR << 1, config3, 2);
}
void initI2C() {
    initIMU();
    initMagnetometer();
}

// Data Conversion
float convertAccel(int16_t raw) {
    return raw * ACCEL_SENSITIVITY;
}
float convertGyro(int16_t raw) {
    return raw * GYRO_SENSITIVITY;
}
float convertMag(int16_t raw) {
    return raw * MAG_SENSITIVITY;
}

// Simple Getters
void readIMUData(uint8_t deviceAddr, uint8_t reg, int16_t &x, int16_t &y, int16_t &z) { // Read 6 bytes from an I2C sensor (X, Y, Z)
    // Note that this assumes that x, y, and z are all of the int16_t datatype
    
    // Tell the I2C to give data, then read that data
    uint8_t data[6];
    uBit.i2c.write(deviceAddr << 1, &reg, 1, true);
    uBit.i2c.read(deviceAddr << 1, data, 6);

    // Get the X, Y, and Z data
    x = (int16_t)((data[1] << 8) | data[0]);
    y = (int16_t)((data[3] << 8) | data[2]);
    z = (int16_t)((data[5] << 8) | data[4]);
}

Vector3 getAccel() {
    // Get the IMU Data
    int16_t raw_x, raw_y, raw_z;
    readIMUData(LSM6DS3TR_C_ADDR, OUTX_L_A, raw_x, raw_y, raw_z);

    // Set the Variables
    Vector3 data;
    data.x = convertAccel(raw_x);
    data.y = convertAccel(raw_y);
    data.z = convertAccel(raw_z);
    return data;
}
Vector3 getGyro() {
    // Get the IMU Data
    int16_t raw_x, raw_y, raw_z;
    readIMUData(LSM6DS3TR_C_ADDR, OUTX_L_G, raw_x, raw_y, raw_z);

    // Set the Variables
    Vector3 data;
    data.x = convertGyro(raw_x);
    data.y = convertGyro(raw_y);
    data.z = convertGyro(raw_z);
    return data;
}
Vector3 getMag() {
    // Get the IMU Data
    int16_t raw_x, raw_y, raw_z;
    readIMUData(LIS3MDL_ADDR, OUTX_L_M, raw_x, raw_y, raw_z);

    // Set the Variables
    Vector3 data;
    data.x = convertMag(raw_x);
    data.y = convertMag(raw_y);
    data.z = convertMag(raw_z);
    return data;
}
IMU getIMU() {
    IMU imuData;
    
    // Get sensor data
    imuData.accel = getAccel();
    imuData.gyro = getGyro();
    imuData.mag = getMag();
    
    return imuData;
}

//////////////////////////////////////////////////////////////////////////////
/*
Current Bugs:
    The data used by the Kalman filter is the i2c data converted from its raw form (int16_t) into it's unit form (float)
        This may cause unexpected errors and increased drift / precision error over time

    Verification of the Kalman Filter has yet to be tested
*/

// Kalman Filter State
State currentState;
State initialState; // Stores the relative reset state

// Kalman Filter Covariance Matrices (Process & Measurement Uncertainty)
Vector3 P_orientation(1, 1, 1); // Process uncertainty (initial estimate)
Vector3 R_orientation(0.01, 0.01, 0.1); // Measurement noise (determines sensor trust level)

// Time step (in seconds)
const float delta = 0.05; // Delta time in seconds

void predict(const IMU& imu) {
	// **Step 1: Predict Orientation using Gyroscope**
	// Gyroscope data provides angular velocity, but it has drift over time.
	// To correct this, we remove the estimated gyroscope bias before integration.
	Vector3 correctedGyro = imu.gyro - currentState.bias;
	
	// Integrate angular velocity over time to estimate new orientation.
	currentState.orientation = currentState.orientation + correctedGyro * delta;

	// **Step 2: Predict Velocity using Accelerometer**
	// Accelerometer measures both linear acceleration and gravity.
	// Since we only need movement acceleration, we subtract gravity.
	Vector3 accelWithoutGravity = imu.accel - Vector3(0, 0, 1); // Assuming gravity is along Z
	
	// Integrate acceleration to estimate velocity.
	currentState.velocity = currentState.velocity + accelWithoutGravity * delta;

	// **Step 3: Predict Position using Velocity**
	// Integrate velocity over time to estimate new position.
	currentState.position = currentState.position + currentState.velocity * delta;

	// **Step 4: Increase Process Uncertainty**
	// As time progresses, estimation error accumulates. We model this increase.
	P_orientation = P_orientation + Vector3(0.001, 0.001, 0.005); // Tuned experimentally
}

void update(const IMU& imu) {
	// **Step 1: Compute the Magnetometer-Based Yaw (Heading)**
	// The magnetometer gives an absolute heading based on Earth's magnetic field.
	// We estimate the yaw angle using arctan.
	float estimatedYaw = atan2(imu.mag.y, imu.mag.x);

	// **Step 2: Compute Kalman Gain for Yaw Correction**
	// Kalman Gain determines how much we trust the new measurement.
	// If uncertainty (P_orientation.z) is high, we trust the magnetometer more.
	// If measurement noise (R_orientation.z) is high, we trust the gyroscope more.
	float K_yaw = P_orientation.z / (P_orientation.z + R_orientation.z);

	// **Step 3: Apply Kalman Gain to Correct Yaw Drift**
	// Instead of a fixed ratio (e.g., 98% gyro, 2% magnetometer), Kalman Gain adapts dynamically.
	currentState.orientation.z = (1 - K_yaw) * currentState.orientation.z + K_yaw * estimatedYaw;

	// **Step 4: Update Process Uncertainty (Covariance Matrix)**
	// After correction, uncertainty is reduced by a factor of (1 - Kalman Gain).
	P_orientation.z = (1 - K_yaw) * P_orientation.z;

	// **Step 5: Adaptive Gyroscope Bias Correction**
	// The gyroscope naturally has drift, which accumulates over time.
	// We use a small gain (K_bias) to gradually estimate and remove this bias.
	float K_bias = 0.01; // Slow adaptation
	currentState.bias = currentState.bias * (1 - K_bias) + imu.gyro * K_bias;
}

State getState() {
	// **Compute State Relative to Reset Point**
	// Since position and orientation are relative, we subtract the initial state.
	State relativeState;
	relativeState.position = currentState.position - initialState.position;
	relativeState.velocity = currentState.velocity - initialState.velocity;
	relativeState.orientation = currentState.orientation - initialState.orientation;
	relativeState.bias = currentState.bias - initialState.bias;
	
	return relativeState;
}

void resetState() {
	// **Store Current State as the New "Zero" Point**
	// This allows position and orientation to be measured relative to a new starting position.
	initialState = currentState;

	// **Reset Process Uncertainty (Start Fresh)**
	P_orientation = Vector3(1, 1, 1); // Reset covariance matrix to initial values
}


//////////////////////////////////////////////////////////////////////////////
// Vector3
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

    // Dot
    uBit.serial.printf("\r\nv1.dot(v2): ");
    printFloat(v1.dot(v2), step);

    // Cross
    uBit.serial.printf("\r\nv1.cross(v2): ");
    printVector3(v1.cross(v2), step);
}

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

//////////////////////////////////////////////////////////////////////////////
int main() {
    // UBit Init
    uBit.init();

    // I2C
    initI2C();

    // Reset State
    //uBit.sleep(2000); // Allow for user position of the bot
    //resetState();

    // While Loop
    while(true) {
        // Get the State of the Bot
        IMU imu = getIMU();
        predict(imu);
        update(imu);
        State state = getState();

        // Print
        uBit.serial.printf("\rIMU:\r\n"); // 1
        printIMU(imu, 2); // 3
        uBit.serial.printf("\r\n\nState:\r\n"); // 1 1
        printState(state, 2); // 4

        // Sleep and Move Cursor Up
        uBit.sleep(delta*1000);
        moveCursorUp(10);
    }
}

