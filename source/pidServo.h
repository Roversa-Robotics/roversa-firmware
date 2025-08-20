#ifndef PID_SERVO_H
#define PID_SERVO_H

#include "MicroBit.h"
#include "helpers.h"
#include "imuFusion.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>

//// MicroBit
extern MicroBit uBit;

//// Servo
#define MAX_US 2300
#define MIN_US 700
#define STOP_US 1500
#define FREQUENCY 50
const int PERIOD = 1000000 / FREQUENCY;;
const int MAX_INPUT = 100;
const int MIN_INPUT = -100;

// Functions
long mapRange(long x, long in_min, long in_max, long out_min, long out_max);
void driveServo(int speedL, int speedR);
void stopServo();

//// PID
// PID Coefficients
const float Kp = 0.1f; // For every degree of offset Yaw, correct the wheel speed by (Kp*100)%
const float Ki = 0.0f;
const float Kd = 0.0f;

// Helpers
float getSmallestDifference(float current_angle, float target_angle);
bool isClockwiseShortest(float current_angle, float target_angle);
float getSignedDifference(float current_angle, float target_angle);
float wrapDegrees180(float angle);

// Operational
void initPID(bool go_forward, int desired_yaw);
void updatePIDServo(bool print = false); // External Update (delta) Function

// PID Driving (External Vars for User-Access)
void forward();
void backward();
void right();
void left();

void stop();

// Helpers
bool is_driving();

#endif // PID_SERVO_H
