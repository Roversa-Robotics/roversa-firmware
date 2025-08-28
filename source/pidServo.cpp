#include "pidServo.h"

/* Future Developments

void updatePIDServo(bool print)
    Refine right() and left() so that they cannot overshoot their desired angle

*/


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
bool GO_FORWARD;
float DESIRED_YAW;

// Do the Drive (When false, don't move)
bool do_drive = false;

// Internal Vars
float pidPreviousError;
float pidIntegral;
float initial_yaw;

// Timing
float drive_prev_time; // msec

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
void initPID(bool go_forward, int desired_yaw) {

    // Basic
    GO_FORWARD = go_forward;
    DESIRED_YAW = desired_yaw;

    // I and D variables
    pidPreviousError = 0;
    pidIntegral = 0;

    // Initial Yaw
    initial_yaw = getYaw();

    // Start the Loop
    drive_prev_time = uBit.systemTime();
    do_drive = true;
}

void updatePIDServo(bool print) { // Return if Loop is Active

    // Do nothing if Inactive
    if (!do_drive) {return;}

    //// Get W
    // Get the Delta
    float drive_current_time = uBit.systemTime(); // in ms
    float delta = (drive_current_time - drive_prev_time) / 1000.0f; // in s
    drive_prev_time = drive_current_time; // in ms

    // Get the Error
    float actual_yaw = wrapDegrees180(getYaw() - initial_yaw);
    float error = getSignedDifference(DESIRED_YAW, actual_yaw);

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

    //// Left and Right
    // If Targetting a Specific yaw, and Once the W Starts impacting the Output, Stop
    if ((DESIRED_YAW != 0.0f) & (std::abs(error) <= 5.0f)) {stop(); return;}

    // Forward
    float left = 100;
    float right = 100;

    // Backwards
    if (!GO_FORWARD) {
        left *= -1;
        right *= -1;
        W *= -1; // Invert W
    }

    // Implement from W
    if (W > 0.0f) {right *= (1.0f - W);} // Value Positive (yawing too much left), Weaken Right Wheel. Else, Weaken Left Wheel
    else {left *= (1.0f + W);}

    // Cap the left and right speeds (so it can only affect ratios between -100% and 100% of the Wheel Speeds)
    if (right > 100.0f) {right = 100.0f;} else if (right < -100.0f) {right = -100.0f;}
    if (left > 100.0f) {left = 100.0f;} else if (left < -100.0f) {left = -100.0f;}

    // Drive
    driveServo(left, right);

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
        printFloat(DESIRED_YAW);
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

        // Delta
        uBit.serial.printf("delta (ms): ");
        printFloat(delta * 1000.0f);
        moveCursorDown(1);
    }
}

// PID Driving (External Vars for User-Access)
void forward() {
    initPID(true, 0);
}

void backward() {
    initPID(false, 0);
}

void right() {
    initPID(true, 90);
}

void left() {
    initPID(true, -90);
}

void stop() {
    do_drive = false;
    stopServo();
}

// Helpers
bool is_driving() {
    return do_drive;
}