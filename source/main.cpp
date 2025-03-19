#include "MicroBit.h"
#include "LSM6DS3Sensor.h"
#include "LIS3MDLSensor.h"

#include <cmath>
#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
// Microbit
MicroBit uBit;
LSM6DS3Sensor imu(&uBit.i2c, 0x6A);
LIS3MDLSensor mag(&uBit.i2c, 0x1E);

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

void printFloat(float value, int step) {
	char buffer[12];
	floatToChar(value, buffer, step);
	uBit.serial.printf("%s", buffer);
}

void moveCursorUp(int lines) {
	for (int i = 0; i < lines; i++) {
		uBit.serial.printf("\x1b[A");
	}
}

//////////////////////////////////////////////////////////////////////////////
// Enumerations for Sensor Sensitivities
enum AccelFS {
	AFS_2G = 2,
	AFS_4G = 4,
	AFS_8G = 8,
	AFS_16G = 16
};

enum GyroFS {
	GFS_125DPS = 125,
	GFS_250DPS = 250,
	GFS_500DPS = 500,
	GFS_1000DPS = 1000,
	GFS_2000DPS = 2000
};

enum MagFS {
	MFS_4GAUSS = 4,
	MFS_8GAUSS = 8,
	MFS_12GAUSS = 12,
	MFS_16GAUSS = 16
};

//////////////////////////////////////////////////////////////////////////////
// Single Sensitivity Setting (Change Here)
const AccelFS ACCEL_SETTING = AFS_2G;
const GyroFS GYRO_SETTING = GFS_125DPS;
const MagFS MAG_SETTING = MFS_4GAUSS;

//////////////////////////////////////////////////////////////////////////////
// Sensor Data Processing
void stateToFloat(int32_t *rawData, float *convertedData, float sensitivity) {
	for (int i = 0; i < 3; i++) {
		convertedData[i] = rawData[i] * sensitivity;
	}
}

void printState(int32_t *rawData, int step, const char* sensorType, const char* unit) {
	uBit.serial.printf("%s (%s): [ ", sensorType, unit);
	for (int i = 0; i < 3; i++) {
		printFloat((float)rawData[i], step);
		uBit.serial.printf(i < 2 ? ", " : " ");
	}
	uBit.serial.printf("]\r\n");
}

void printIMU(int step) {
	int32_t rawData[3];
	float convertedData[3];

	imu.Get_X_Axes(rawData);
	stateToFloat(rawData, convertedData, LSM6DS3_ACC_SENSITIVITY_FOR_FS_2G);
	printState(rawData, step, "Accel", "m/s^2");

	imu.Get_G_Axes(rawData);
	stateToFloat(rawData, convertedData, LSM6DS3_GYRO_SENSITIVITY_FOR_FS_125DPS);
	printState(rawData, step, "Gyro", "deg/s");

	mag.Get_M_Axes(rawData);
	stateToFloat(rawData, convertedData, LSM6DS3_MAG_SENSITIVITY_FOR_FS_4GAUSS); // TODO: CHANGE LATER
	printState(rawData, step, "Mag", "uT");
}

//////////////////////////////////////////////////////////////////////////////
// Main
int main() {
	uBit.init();

	if (imu.begin() != 0) {
		uBit.serial.printf("IMU initialization failed\r\n");
		return 1;
	}
	if (mag.begin() != 0) {
		uBit.serial.printf("Magnetometer initialization failed\r\n");
		return 1;
	}

	imu.Set_X_FS(ACCEL_SETTING);
	imu.Set_G_FS(GYRO_SETTING);
	mag.Set_M_FS(MAG_SETTING);

	imu.Enable_X();
	imu.Enable_G();
	mag.Enable();

	while (true) {
		printIMU(2);
		uBit.sleep(10);
	}
}
