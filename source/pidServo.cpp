#include "pidServo.h"


//// MicroBit
extern MicroBit uBit;


//// Servo
long mapRange(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveServo(int speedL, int speedR) {                                              //-100-100 each function will take care of wheel direction - 0 is neutral

    /*
    drive(0,0); neutral
    drive(100,100); //forward
    drive(-100,-100); //reverse
    drive(-100,100); //left turn
    drive(100,-100); //right turn
    */

    int angleSpeedL = mapRange(speedL,MIN_INPUT,MAX_INPUT,0,180);                 //user decides direction of motor L motor is CCW for forward
    int angleSpeedR = mapRange(speedR,MIN_INPUT,MAX_INPUT,180,0);                 //user decides direction of motor R motor is CW for forward
    uBit.io.P1.setServoPulseUs(PERIOD);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(PERIOD);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(angleSpeedL, MAX_US, STOP_US);                       //left motor
    uBit.io.P2.setServoValue(angleSpeedR, MAX_US, STOP_US);                       //right motor
}

void stopServo() {
    uBit.io.P1.setDigitalValue(0);                                                //left motor GPIO low - no signal to motor
    uBit.io.P2.setDigitalValue(0);                                                //right motor GPIO low - no signal to motor
    uBit.io.P1.setServoPulseUs(PERIOD);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(PERIOD);                                           //set right motor period for analog
}


//// PID
// Init
int LEFT;
int RIGHT;
bool do_drive = false;

// Internal Vars
float pidPreviousError;
float pidIntegral;
float initial_yaw;

// Timing
float drive_start_time; // msec
float drive_end_time; // msec

// Helpers
float getSmallestDifference(float current_angle, float target_angle) {
    float angle_diff = target_angle - current_angle + 180.0f;
    float wrapped = std::fmod(angle_diff, 360.0f);
    if (wrapped < 0) wrapped += 360.0f;
    return std::abs(wrapped - 180.0f);
}

bool isClockwiseShortest(float current_angle, float target_angle) {
    float delta = std::fmod((target_angle - current_angle + 360.0f), 360.0f);
    return (delta > 180.0f);
}

float getSignedDifference(float current_angle, float target_angle) {
    float difference = getSmallestDifference(current_angle, target_angle);
    bool clockwise = isClockwiseShortest(current_angle, target_angle);
    return clockwise ? difference : -difference;
}

float wrapDegrees180(float angle) {
    return std::remainder(angle, 360.0f);
}

// Operational
void initPID(int new_LEFT, int new_RIGHT) {

    // Basic
    LEFT = new_LEFT;
    RIGHT = new_RIGHT;

    // I and D variables
    pidPreviousError = 0;
    pidIntegral = 0;

    // Initial Yaw
    initial_yaw = getYaw();

    // Start the Loop
    drive_start_time = uBit.systemTime();
    drive_end_time = drive_start_time;
    do_drive = true;
}

void updatePIDServo(bool print) { // Return if Loop is Active

    // Do nothing if Inactive
    if (!do_drive) {return;}

    //// Get W
    // Get the Error
    float desired_yaw = 0; // TODO: Automate
    float actual_yaw = wrapDegrees180(getYaw() - initial_yaw);
    float error = getSignedDifference(desired_yaw, actual_yaw);
    float delta = getDelta(); // From the Fusion Algorithm

    // Basic
    pidIntegral += error * delta;
    float derivative = (error - pidPreviousError) / delta;
    pidPreviousError = error;

    // P (proportional)
    float P = Kp * error;

    // I (integral)
    float I = Ki * pidIntegral;

    // D (derivative)
    float D = Kd * derivative;
    
    // W (P + I + D)
    float W = P + I + D;
    float left = LEFT;
    float right = RIGHT;
    if (W > 0.0f) {right *= (1.0f - W);} // Value Positive (yawing too much left), Weaken Right Wheel. Else, Weaken Left Wheel
    else {left *= (1.0f + W);}

    //// Drive based on W
    // Cap the left and right speeds (so it can only affect ratios between -100% and 100% of the Wheel Speeds)
    if (right > 100.0f) {right = 100.0f;} else if (right < -100.0f) {right = -100.0f;}
    if (left > 100.0f) {left = 100.0f;} else if (left < -100.0f) {left = -100.0f;}

    // Drive
    driveServo(left, right);

    // Drive End Time
    drive_end_time = uBit.systemTime();

    // Print
    if (print) {

        // Yaw
        uBit.serial.printf("yaw: [");

        uBit.serial.printf("initial: ");
        printFloat(initial_yaw);
        uBit.serial.printf(", ");

        uBit.serial.printf("measured: ");
        printFloat(getYaw());
        uBit.serial.printf(", ");

        uBit.serial.printf("actual (m - i): ");
        printFloat(actual_yaw);
        uBit.serial.printf(", ");

        uBit.serial.printf("desired: ");
        printFloat(desired_yaw);
        uBit.serial.printf(", ");

        uBit.serial.printf("error (d - a): ");
        printFloat(error);
        uBit.serial.printf("]");

        moveCursorDown(2);
        
        // W = P + I + D
        uBit.serial.printf("W: ");
        printFloat(W);
        uBit.serial.printf(" = sum(");

        uBit.serial.printf("P: ");
        printFloat(P);
        uBit.serial.printf(", ");

        uBit.serial.printf("I: ");
        printFloat(I);
        uBit.serial.printf(", ");

        uBit.serial.printf("D: ");
        printFloat(D);
        uBit.serial.printf(")");
        
        moveCursorDown(1);

        // Left and Right
        uBit.serial.printf("left, right: [");
        printFloat(left);
        uBit.serial.printf(", ");
        printFloat(right);
        uBit.serial.printf("]");
        moveCursorDown(1);

        // Time Drive
        uBit.serial.printf("time driven (ms): ");
        printFloat(get_time_driven());
        moveCursorDown(1);
    }
}

// PID Driving (External Vars for User-Access)
void forward() {
    initPID(100, 100);
}

void stop() {
    do_drive = false;
    stopServo();
}

bool is_driving() {
    return do_drive;
}

float get_time_driven() { // In msec
    return (drive_end_time - drive_start_time);
}