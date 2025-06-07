#include "imuFusion.h"

//// MicroBit
extern MicroBit uBit;


//// IMU
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

void printAxes(float* axes, int step) {

    // Print
    uBit.serial.printf("[");
    for (int i = 0; i < 3; i++) {
        printFloat(axes[i], step);
        uBit.serial.printf(i < 2 ? ", " : "]");
    }
}

// Getters
void getAcc(float* axes, float scalar) {
    // Raw Units: mg
    int32_t raw_axes[3];
    accgyro.Get_X_Axes(raw_axes);
    scaleAxes(axes, raw_axes, scalar);
    axesToNed(axes);
}

void getGyro(float* axes, float scalar) {
    // Raw Units: mdps
    int32_t raw_axes[3];
    accgyro.Get_G_Axes(raw_axes);
    scaleAxes(axes, raw_axes, scalar);
    axesToNed(axes);
}

void getMag(float* axes, float scalar) {
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
    moveCursorDown(1);

    uBit.serial.printf("Gyro (dps): ");
    printAxes(temp_gyro);
    moveCursorDown(1);

    uBit.serial.printf("Mag (Gauss): ");
    printAxes(temp_mag);
    moveCursorDown(1);
}


//// Fusion
// Fusion Timing
unsigned int SAMPLE_RATE;
float fusionDeltaTime;
static unsigned long fusionPreviousTime;

// Fusion Output
FusionEuler fusionEuler;
FusionVector fusionEarth;

// State
FusionOffset fusionOffset;
FusionAhrs fusionAHRS;

// Init
void initFusion(unsigned int new_SAMPLE_RATE) {
    // Set the Sample Rate
    SAMPLE_RATE = new_SAMPLE_RATE;

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

// Getters
float getAxisX() { // In "Gravity Compensated" gravities
    return fusionEarth.axis.x;
}
float getAxisY() { // In "Gravity Compensated" gravities
    return fusionEarth.axis.y;
}
float getAxisZ() { // In "Gravity Compensated" gravities
    return fusionEarth.axis.z;
}

float getRoll() { // In degrees
    return fusionEuler.angle.roll;
}
float getPitch() { // In degrees
    return fusionEuler.angle.pitch;
}
float getYaw() { // In degrees
    return fusionEuler.angle.yaw;
}

float getDelta() { // In Seconds
    return fusionDeltaTime;
}

// Printing
void printFusion() {
    // DeltaTime
    uBit.serial.printf("Delta (ms): ");
    printFloat(getDelta() * 1000.0f, 3);
    moveCursorDown(1);

    // Acceleration (Gravity Compensated)
    uBit.serial.printf("Acceleration (Gravity Compensated) (gravities): [");
    printFloat(getAxisX(), 3);
    uBit.serial.printf(", ");
    printFloat(getAxisY(), 3);
    uBit.serial.printf(", ");
    printFloat(getAxisZ(), 3);
    uBit.serial.printf("]");
    moveCursorDown(1);

    // Orientation
    uBit.serial.printf("Orientation (degrees): [");
    printFloat(getRoll(), 3);
    uBit.serial.printf(", ");
    printFloat(getPitch(), 3);
    uBit.serial.printf(", ");
    printFloat(getYaw(), 3);
    uBit.serial.printf("]");
    moveCursorDown(1);
}

//// IMUFusion
void initIMUFusion(unsigned int new_SAMPLE_RATE) {
    initIMU();
    initFusion(new_SAMPLE_RATE);
}

void updateIMUFusion() {
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
     fusionDeltaTime = (currentTime - fusionPreviousTime) / 1000.0f; // Convert ms to seconds
     fusionPreviousTime = currentTime;

     // Update gyroscope AHRS algorithm
     FusionAhrsUpdate(&fusionAHRS, gyroscope, accelerometer, magnetometer, fusionDeltaTime);

     // Print algorithm outputs
     fusionEuler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusionAHRS));
     fusionEarth = FusionAhrsGetEarthAcceleration(&fusionAHRS);
}

void printIMUFusion() {
    // Print
    printIMU(); // 3 Lines
    moveCursorDown(1); // 1 Lines
    printFusion(); // 3 Lines
}