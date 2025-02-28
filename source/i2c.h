#ifndef I2C_H
#define I2C_H

#include "helpers.h"

// I2C Addresses
#define LSM6DS3TR_C_ADDR 0x6A // Accelerometer & Gyroscope
#define LIS3MDL_ADDR 0x1E // Magnetometer

// Register addresses
#define CTRL1_XL 0x10 // Accelerometer control
#define CTRL2_G 0x11 // Gyroscope control
#define OUTX_L_A 0x28 // Accelerometer X low byte
#define OUTX_L_G 0x22 // Gyroscope X low byte
#define OUTX_L_M 0x28 // Magnetometer X low byte

// Misc
#define ROUND_STEPS 2 // Number of decimal places for accelerometer

// Function Prototypes
void initIMU();
void initMagnetometer();
void readIMUData(uint8_t deviceAddr, uint8_t reg, int16_t &x, int16_t &y, int16_t &z);
float convertAccel(int16_t raw);
float convertGyro(int16_t raw);
float convertMag(int16_t raw);
Vector3 getAccel();
Vector3 getGyro();
Vector3 getMag();

#endif // I2C_H
