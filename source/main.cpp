#include "MicroBit.h"
#include "LIS3MDLSensor.h"
#include "LSM6DS3Sensor.h"

#include "Fusion/Fusion.h"

#include "servoRoversa.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>

//// Microbit
MicroBit uBit;
bool buttonAWasPressed = false;
bool buttonBWasPressed = false;


//// Helper Functions
float round(float value, int step) {

    float multiplier = 1;
    for (int i = 0; i < step; i++) multiplier *= 10;
    return (int)(value * multiplier + 0.5f) / multiplier;
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
    do { temp /= 10; digits++; } while (temp > 0);

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

void printFloat(float value, int step = 2) {
    char buffer[12];
    floatToChar(value, buffer, step);
    uBit.serial.printf("%s", buffer);
}

void moveCursorUp(int lines) {
    for (int i = 0; i < lines; i++) {
        uBit.serial.printf("\x1b[A");   // Move cursor up
        uBit.serial.printf("\r\x1b[2K"); // Return to start of line and clear it
    }
}

void moveCursorDown(int lines) {
    for (int i = 0; i < lines; i++) {
        uBit.serial.printf("\r\n");   // Move cursor down
    }
}


//// IMU
// Vars
LSM6DS3Sensor accgyro(&uBit.i2c, 0x6A);
LIS3MDLSensor mag(&uBit.i2c, 0x1E);

// Setup
void initIMU() {
    // Gyroscope and Accelerometer
    if (accgyro.begin() != 0) {
        uBit.serial.printf("Accelerometer / Gyroscope initialization failed\r\n");
        return;
    }
    accgyro.Enable_X();
    accgyro.Enable_G();

    // Magnetometer
    if (mag.begin() != 0) {
        uBit.serial.printf("Magnetometer initialization failed\r\n");
        return;
    }
    mag.Enable_M();
}

// Operational
void scaleAxes(float* axes, int32_t* raw_axes, float scalar) {
    for (int i = 0; i < 3; i++) {
        axes[i] = ((float)raw_axes[i]) * scalar;
    }
}

void axesToNed(float* axes) { // Converts IMU to Fusion-Usable Data
    float x = axes[0];
    float y = axes[1];
    float z = axes[2];
    axes[0] = -y;      // North  = -South
    axes[1] = -x;      // East   = -West
    axes[2] = z; // Down stays Down
}

void printAxes(float* axes, int step = 2) {

    // Print
    uBit.serial.printf("[");
    for (int i = 0; i < 3; i++) {
        printFloat(axes[i], step);
        uBit.serial.printf(i < 2 ? ", " : "]");
    }
}

// Getters
void getAcc(float* axes, float scalar = 1e-3f) {
    // Raw Units: mg
    int32_t raw_axes[3];
    accgyro.Get_X_Axes(raw_axes);
    scaleAxes(axes, raw_axes, scalar);
    axesToNed(axes);
}

void getGyro(float* axes, float scalar = 1e-3f) {
    // Raw Units: mdps
    int32_t raw_axes[3];
    accgyro.Get_G_Axes(raw_axes);
    scaleAxes(axes, raw_axes, scalar);
    axesToNed(axes);
}

void getMag(float* axes, float scalar = 1e-3f) {
    // Raw Units: mGauss
    int32_t raw_axes[3];
    mag.Get_M_Axes(raw_axes);
    scaleAxes(axes, raw_axes, scalar);
    axesToNed(axes);
}

void printIMU() {
    // Get the Axes
    float temp_acc[3];
    getAcc(temp_acc);

    float temp_gyro[3];
    getGyro(temp_gyro);

    float temp_mag[3];
    getMag(temp_mag);

    // Print the Axes
    uBit.serial.printf("Acc (g): ");
    printAxes(temp_acc);
    uBit.serial.printf("\r\n");

    uBit.serial.printf("Gyro (dps): ");
    printAxes(temp_gyro);
    uBit.serial.printf("\r\n");

    uBit.serial.printf("Mag (Gauss): ");
    printAxes(temp_mag);
    uBit.serial.printf("\r\n");
}


//// Fusion
// Error
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {-0.97f, -1.76f, 1.165f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.045f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Timing
#define SAMPLE_RATE (100) // Samples Per Second (aka Hz) (Hz = 1 / SAMPLES)
static unsigned long fusionPreviousTime;

// Internal Vars
FusionOffset fusionOffset;
FusionAhrs fusionAHRS;

// External Vars
float fusionDeltaTime;
FusionEuler fusionEuler;
FusionVector fusionEarth;

// Setup
void initFusion() {

    // Initialise algorithms
    FusionOffsetInitialise(&fusionOffset, SAMPLE_RATE);
    FusionAhrsInitialise(&fusionAHRS);

    // Gyro Sensitivty
    float gyro_sensitivity;
    accgyro.Get_G_Sensitivity(&gyro_sensitivity);
    gyro_sensitivity *= 1000; // Convert from mdps to dps

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNed, // DEF: North, East, Down
            .gain = 0.5f, // DEF: Controls how strongly the AHRS algorithm trusts accelerometer and magnetometer feedback when correcting orientation drift.
            .gyroscopeRange = gyro_sensitivity, // DEF: replace this with actual gyroscope range in degrees/s // TODO: Start here, easy data to find
            .accelerationRejection = 10.0f, // DEF: What this means: “If the accelerometer vector differs from expected gravity by more than 10 degrees, do not trust it for this update.”
            .magneticRejection = 10.0f, // DEF: Similar for Above
            .recoveryTriggerPeriod = 5 * SAMPLE_RATE, // DEF: “If the accelerometer/magnetometer got ignored due to bad data, how long do we wait before letting it influence the orientation again?” (in terms of Seconds as units)
    };
    FusionAhrsSetSettings(&fusionAHRS, &settings);

    // Previous Time
    fusionPreviousTime = uBit.systemTime();
}

// Operational
void updateFusion() {
    /*
    Units (overall)
        Accelerometer: Gravitational Units (g)
        Gyroscope: Degrees Per Second (dps)
        Magnetometer: Any
        DeltaTime: Seconds
    */

     // Acquire latest sensor data
     float temp_acc[3];
     getAcc(temp_acc);
     FusionVector accelerometer = {temp_acc[0], temp_acc[1], temp_acc[2]};

     float temp_gyro[3];
     getGyro(temp_gyro);
     FusionVector gyroscope = {temp_gyro[0], temp_gyro[1], temp_gyro[2]};

     float temp_mag[3];
     getMag(temp_mag);
     FusionVector magnetometer = {temp_mag[0], temp_mag[1], temp_mag[2]};

     // Apply calibration
     accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
     gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
     magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

     // Update gyroscope offset correction algorithm
     gyroscope = FusionOffsetUpdate(&fusionOffset, gyroscope);

     // Calculate delta time (in seconds) to account for gyroscope sample clock error
     unsigned long currentTime = uBit.systemTime();
     float fusionDeltaTime = (currentTime - fusionPreviousTime) / 1000.0f; // Convert ms to seconds
     fusionPreviousTime = currentTime;

     // Update gyroscope AHRS algorithm
     FusionAhrsUpdate(&fusionAHRS, gyroscope, accelerometer, magnetometer, fusionDeltaTime);

     // Print algorithm outputs
     fusionEuler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusionAHRS));
     fusionEarth = FusionAhrsGetEarthAcceleration(&fusionAHRS);
}

void printFusion() {
    // DeltaTime
    uBit.serial.printf("Delta (ms): ");
    printFloat(fusionDeltaTime * 1000.0f, 3);
    uBit.serial.printf("\r\n");

    // Acceleration (Gravity Compensated)
    uBit.serial.printf("Acceleration (Gravity Compensated) (gravities): [");
    printFloat(fusionEarth.axis.x, 3);
    uBit.serial.printf(", ");
    printFloat(fusionEarth.axis.y, 3);
    uBit.serial.printf(", ");
    printFloat(fusionEarth.axis.z, 3);
    uBit.serial.printf("]\r\n");

    // Orientation
    uBit.serial.printf("Orientation (degrees): [");
    printFloat(fusionEuler.angle.roll, 3);
    uBit.serial.printf(", ");
    printFloat(fusionEuler.angle.pitch, 3);
    uBit.serial.printf(", ");
    printFloat(fusionEuler.angle.yaw, 3);
    uBit.serial.printf("]\r\n");
}


//// PID
// Constants
const float Kp = 1.0f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// Internal Vars
float pidPreviousError;
float pidIntegral;
float initial_yaw;

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

// Setup
void initPID() {
    // Basic
    pidPreviousError = 0;
    pidIntegral = 0;

    // Get the Initial Yaw
    initial_yaw = fusionEuler.angle.yaw;
}

// Operational
float updatePID(float desired_yaw, float delta, bool print = false) { // Returns the Speed Parameter
    // Get the Error
    float actual_yaw = wrapDegrees180(fusionEuler.angle.yaw - initial_yaw);
    float error = getSignedDifference(desired_yaw, actual_yaw);

    // P (proportional)
    float P = Kp * error;

    // I (integral)
    pidIntegral += error * delta;
    float I = Ki * pidIntegral;

    // D (derivative)
    float derivative = (error - pidPreviousError) / delta;
    float D = Kd * derivative;
    pidPreviousError = error;

    // W (P + I + D)
    float W = P + I + D;

    // Print
    if (print) {
        
        // Cursor Up
        moveCursorUp(7);

        // Yaw Values
        uBit.serial.printf("initial_yaw: ");
        printFloat(initial_yaw);
        moveCursorDown(1);

        uBit.serial.printf("desired_yaw: ");
        printFloat(desired_yaw);
        moveCursorDown(1);

        uBit.serial.printf("actual_yaw: ");
        printFloat(actual_yaw);
        moveCursorDown(1);

        uBit.serial.printf("error_yaw: ");
        printFloat(error);
        moveCursorDown(1);
        
        // W = P + I + D
        moveCursorDown(1);

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
    }

    // Return
    return W;
}

void drivePID(float left, float right, float expected_yaw, bool print = false) {

    // Get the W
    float W = updatePID(expected_yaw, fusionDeltaTime, print);

    // Drive
    float true_left = left;
    float true_right = right;

    // Drive
    drive(true_left, true_right);
}


//// Main
// Constants
const float DELTA = 1.0f/SAMPLE_RATE; // In Seconds (0.1 s currently)

// Buttons
static void onButtonA(MicroBitEvent) {
    buttonAWasPressed = true;
}

static void onButtonB(MicroBitEvent) {
    buttonBWasPressed = true;
}


//// Tests
void testFusion() {
    
    // Init
    initIMU();
    initFusion();

    // Print Statement
    uBit.serial.printf("\r\nSetup Complete: Press A to Continue");

    // While Loop
    while (true) {

        // Do Action
        if (buttonAWasPressed) {
            // Fusion Update
            updateFusion();

            // Print
            moveCursorUp(8);
            printIMU(); // 3 Lines
            moveCursorDown(2); // 2 Lines
            printFusion(); // 3 LinesW
        }

        // Wait
        uBit.sleep(DELTA * 1000);
    }
}

void testPID() {

    //// External Variables
    // Driving
    const float LEFT = 100.0f;
    const float RIGHT = 100.0f;
    const float EXPECTED_YAW = 0.0f;

    // Timing
    const float FUSION_STABILIZE_TIME = 2000.0f; // In ms
    const float PID_TIME = 2000.0f; // In ms


    //// Internal Variables
    // Basic
    bool fusionStabilized = false;
    bool started = false;
    bool ignorePID;

    // Timing
    unsigned long start_time = uBit.systemTime();; // In ms
    unsigned long end_time; // In ms


    //// Init
    initIMU();
    initFusion();
    stop(); // stop driving

    // Print Statement
    moveCursorDown(1);
    uBit.serial.printf("Stabilizing the Fusion Algorithm...");
    moveCursorDown(1);


    //// While Loop
    while (true) {
        
        // Fusion not Stabilized
        if (!fusionStabilized) {

            // Trigger, Switch to Start Sequence
            end_time = uBit.systemTime();
            if ((end_time - start_time) >= FUSION_STABILIZE_TIME) {
                fusionStabilized = true;

                // Print Statement
                moveCursorDown(1);
                uBit.serial.printf("Fusion Stabilization Complete:");
                moveCursorDown(1);
                uBit.serial.printf("Press A to run with the PID Loop");
                moveCursorDown(1);
                uBit.serial.printf("Press B to run without the PID Loop");
                moveCursorDown(1);
            }
        }

        // Drive Loop Started
        else if (started) {

            // Triggered by Time Elapsed: End
            end_time = uBit.systemTime();
            if ((end_time - start_time) >= PID_TIME) {

                // Basic
                buttonAWasPressed = false;
                buttonBWasPressed = false;
                started = false;

                // Stop Driving
                stop();
            }

            // Drive Forwards
            else {

                // Use PID
                if (!ignorePID) {drivePID(LEFT, RIGHT, EXPECTED_YAW, true);}

                // Ignore PID
                else {drive(LEFT, RIGHT);}
            }
        }

        // Reset
        else if (buttonAWasPressed || buttonBWasPressed) {
            // Basic
            started = true;
            start_time = uBit.systemTime();
            ignorePID = buttonBWasPressed;

            // Init
            initPID();

            // Stop Driving
            stop();
        }

        // Update Fusion
        updateFusion();

        // Wait
        uBit.sleep(DELTA * 1000);
    }
}

/*
void PIDTest1() {
    // Vars
    bool start = false;

    // Init
    initIMU();

    // While Loop
    while (true) {

         //// Ignore
         if (!buttonAWasPressed) {}
        
         //// Init
         else if !(start) {
            start = true;

            // Init the Fusion
            initFusion();

            // Setup the PID
            initPID()

            // Start the Servo
            forward(100, 100);

         }

         //// Update
         // Update Fusion
         updateFusion();

         // Update the PID

        // Wait
        uBit.sleep(DELTA * 1000);
    }
}
*/

//// Main
int main() {

    // UBit Setup
    uBit.init();
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, onButtonB);

    // Test
    testPID();
}