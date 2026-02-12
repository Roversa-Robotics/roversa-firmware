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


/* Future Developments (not yet tested):
How to Check if System has Stablized:
    Using the Fusion Folder, run
        FusionAhrsGetFlags(const FusionAhrs *const ahrs);
    
    it will Return
        typedef struct {
            bool initialising;
            bool angularRateRecovery;
            bool accelerationRecovery;
            bool magneticRecovery;
        } FusionAhrsFlags;
    
    If all of the booleans are false, then the system has stabilized
        FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs);
        bool isStable = !(flags.initialising || flags.angularRateRecovery || flags.accelerationRecovery || flags.magneticRecovery);
    
    Notes
        The I2C cpp and h files do not contain explicit definitions for a "stationary" robot, Fusion is better for this task.

Additional Inputs
    In LSM6DS3Sensor, there are several functions that allow for additional inputs to be detected by the bot using the Gyroscope and Accelerometer, such as...
        Free Fall Detection
        Getting Pedometer Steps
        Tilt Detection
        Wake up Detection
        Single Tap Detection
        Double Tap Detection
    
    These can be implemented to detect additional inputs delivered to the bot

*/


//// MicroBit
extern MicroBit uBit;


//// IMU
// Vars
extern LSM6DS3Sensor accgyro;
extern LIS3MDLSensor mag;

// Setup
void scanI2CBus();
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
// NOTE: Below are hard-coded values taken through experimentation, TODO: Check if can be corrected in the PID Loop

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
