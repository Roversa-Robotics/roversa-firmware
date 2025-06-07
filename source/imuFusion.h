#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "MicroBit.h"
#include "helpers.h"

#include "LIS3MDLSensor.h"
#include "LSM6DS3Sensor.h"

#include "Fusion/Fusion.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>


//// MicroBit
extern MicroBit uBit;


//// IMU
// Vars
extern LSM6DS3Sensor accgyro;
extern LIS3MDLSensor mag;

// Setup
void initIMU();

// Operations
void scaleAxes(float* axes, int32_t* raw_axes, float scalar);
void axesToNed(float* axes);
void printAxes(float* axes, int step = 2);

// Getters
void getAcc(float* axes, float scalar = 1e-3f);
void getGyro(float* axes, float scalar = 1e-3f);
void getMag(float* axes, float scalar = 1e-3f);
void printIMU();


//// Fusion
// Alignment
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {-0.97f, -1.76f, 1.165f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.045f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Timing
extern unsigned int SAMPLE_RATE; // Samples Per Second (aka Hz) (Hz = 1 / SAMPLES)
extern float fusionDeltaTime; // In Seconds

// State
extern FusionEuler fusionEuler;
extern FusionVector fusionEarth;

// Init
void initFusion(unsigned int new_SAMPLE_RATE);

// Getters
float getAxisX();
float getAxisY();
float getAxisZ();

float getRoll();
float getPitch();
float getYaw();

float getDelta();

// Printing
void printFusion();


//// IMUFusion
void initIMUFusion(unsigned int new_SAMPLE_RATE);
void updateIMUFusion(); // The same as "updateFusion"
void printIMUFusion();

#endif // IMU_FUSION_H
