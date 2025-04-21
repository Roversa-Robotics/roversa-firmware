#include "MicroBit.h"
#include "LIS3MDLSensor.h"
#include "LSM6DS3Sensor.h"

#include "Fusion/Fusion.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>

//// Vars
// Microbit
MicroBit uBit;
bool startStream = false;

// IMU
LSM6DS3Sensor accgyro(&uBit.i2c, 0x6A);
LIS3MDLSensor mag(&uBit.i2c, 0x1E);

// Fusion
#define SAMPLE_RATE (100) // Samples Per Second (aka Hz) (Hz = 1 / SAMPLES)
const float DELTA = 0.01; // In Seconds

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

void printFloat(float value, int step) {
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

//// IMU
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

// Operators
void scaleAxes(float* axes, int32_t* raw_axes, float scalar) {
    for (int i = 0; i < 3; i++) {
        axes[i] = ((float)raw_axes[i]) * scalar;
    }
}

void axesToNed(float* axes) {
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

// Fusion // TODO
void initFusion() {

}

void updateFusion() {

}

void predictFusion() {
    
}

// Fusion
/*
Units (overall)
    Accelerometer: Gravitational Units (g)
    Gyroscope: Degrees Per Second (dps)
    Magnetometer: Any
    DeltaTime: Seconds
*/

// Button A event handler
static void onButtonA(MicroBitEvent) {
    // calibrate();
    startStream = true;
}

int main() {
    //// Basic
    uBit.init();
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);

    //// Fusion Setup
    
    // Define calibration (replace with actual calibration data if available)
    /*
    Sensitivites do not need to be changed (since we aren't pumping in the RAW values, but the actual ones)
    */
   // FusionMatrixMultiplyVector(misalignment, FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {-0.69f, -1.74f, 1.12f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.05f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

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
    FusionAhrsSetSettings(&ahrs, &settings);

    //// IMU Setup
    initIMU();
    uBit.serial.printf("IMU initialized. Press A to start streaming.");

    //// While Loop
    static unsigned long previousTime = uBit.systemTime();
    while (true) {

        // Do Action
        if (startStream) {
            //// Fusion Algorithm
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
            gyroscope = FusionOffsetUpdate(&offset, gyroscope);

            // Calculate delta time (in seconds) to account for gyroscope sample clock error
            unsigned long currentTime = uBit.systemTime();
            float deltaTime = (currentTime - previousTime) / 1000.0f; // Convert ms to seconds
            previousTime = currentTime;

            // Update gyroscope AHRS algorithm
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

            // Print algorithm outputs
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

            //// Print
            // Setup
            moveCursorUp(8);

            // IMU
            // Accelerometer
            uBit.serial.printf("Acc (g): ");
            printAxes(temp_acc);
            uBit.serial.printf("\r\n");

            // Gyroscope
            uBit.serial.printf("Gyro (dps): ");
            printAxes(temp_gyro);
            uBit.serial.printf("\r\n");

            // Magnetometer
            uBit.serial.printf("Mag (Gauss): ");
            printAxes(temp_mag);
            uBit.serial.printf("\r\n");

            // Fusion
            uBit.serial.printf("\r\n");

            uBit.serial.printf("Delta (s): ");
            printFloat(deltaTime, 3);
            uBit.serial.printf("\r\n");

            uBit.serial.printf("Acceleration (Gravity Compensated) (gravities): [");
            printFloat(earth.axis.x, 3);
            uBit.serial.printf(", ");
            printFloat(earth.axis.y, 3);
            uBit.serial.printf(", ");
            printFloat(earth.axis.z, 3);
            uBit.serial.printf("]\r\n");

            uBit.serial.printf("Orientation (degrees): [");
            printFloat(euler.angle.roll, 3);
            uBit.serial.printf(", ");
            printFloat(euler.angle.pitch, 3);
            uBit.serial.printf(", ");
            printFloat(euler.angle.yaw, 3);
            uBit.serial.printf("]\r\n");

            //// Wait
            uBit.sleep(DELTA * 1000);
    
        // Wait
        } else {
            uBit.sleep(DELTA * 1000);
        }
    }
}

/*
void printIMU(int step = 3) {

    // Accelerometer
    uBit.serial.printf("Acc (g): ");
    float accel[3];
    getAcc(accel);
    printAxes(accel, step);
    uBit.serial.printf("\r\n");

    // Gyroscope
    uBit.serial.printf("Gyro (dps): ");
    float gyro[3];
    getGyro(gyro);
    printAxes(gyro, step);
    uBit.serial.printf("\r\n");

    // Magnetometer
    uBit.serial.printf("Mag (Gauss): ");
    float mag[3];
    getMag(mag);
    printAxes(mag, step);
    uBit.serial.printf("\r\n");
}
*/