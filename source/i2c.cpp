#include "MicroBit.h"
#include "i2c.h"

// Variables
extern MicroBit uBit;

/* I2C */
void initIMU() { // Initialize Accelerometer, Gyroscope
    uint8_t accelConfig[2] = {CTRL1_XL, 0x60}; // 208Hz ODR, ±2g
    uint8_t gyroConfig[2]  = {CTRL2_G,  0x60}; // 208Hz ODR, 250 dps

    uBit.i2c.write(LSM6DS3TR_C_ADDR << 1, accelConfig, 2);
    uBit.i2c.write(LSM6DS3TR_C_ADDR << 1, gyroConfig, 2);
}

void initMagnetometer() { // Initialize the Magnetometer
    uint8_t config1[2] = {0x20, 0x70}; // Enable magnetometer, 10Hz ODR, continuous mode
    uint8_t config2[2] = {0x21, 0x00}; // Default scale ±4 Gauss
    uint8_t config3[2] = {0x22, 0x00}; // Continuous-conversion mode

    uBit.i2c.write(LIS3MDL_ADDR << 1, config1, 2);
    uBit.i2c.write(LIS3MDL_ADDR << 1, config2, 2);
    uBit.i2c.write(LIS3MDL_ADDR << 1, config3, 2);
}

void readIMUData(uint8_t deviceAddr, uint8_t reg, int16_t &x, int16_t &y, int16_t &z) { // Read 6 bytes from an I2C sensor (X, Y, Z)
    // Note that this assumes that x, y, and z are all of the int16_t datatype
    
    // Tell the I2C to give data, then read that data
    uint8_t data[6];
    uBit.i2c.write(deviceAddr << 1, &reg, 1, true);
    uBit.i2c.read(deviceAddr << 1, data, 6);

    // Get the X, Y, and Z data
    x = (int16_t)((data[1] << 8) | data[0]);
    y = (int16_t)((data[3] << 8) | data[2]);
    z = (int16_t)((data[5] << 8) | data[4]);
}

/* Data Conversion */
float convertAccel(int16_t raw) { // Convert raw accelerometer data to `g`
    return raw * 0.000061;
}

float convertGyro(int16_t raw) { // Convert raw gyroscope data to `dps`
    return raw * 0.00875;
}

float convertMag(int16_t raw) { // Convert raw magnetometer data to microteslas (uT)
    return raw * 0.00014;
}

// Simple Getters
Vector3 getAccel() {
    // Get the IMU Data
    int16_t raw_x, raw_y, raw_z;
    readIMUData(LSM6DS3TR_C_ADDR, OUTX_L_A, raw_x, raw_y, raw_z);

    // Set the Variables
    Vector3 data;
    data.x = convertAccel(raw_x);
    data.y = convertAccel(raw_y);
    data.z = convertAccel(raw_z);
    return data;
}

Vector3 getGyro() {
    // Get the IMU Data
    int16_t raw_x, raw_y, raw_z;
    readIMUData(LSM6DS3TR_C_ADDR, OUTX_L_G, raw_x, raw_y, raw_z);

    // Set the Variables
    Vector3 data;
    data.x = convertGyro(raw_x);
    data.y = convertGyro(raw_y);
    data.z = convertGyro(raw_z);
    return data;
}

Vector3 getMag() {
    // Get the IMU Data
    int16_t raw_x, raw_y, raw_z;
    readIMUData(LIS3MDL_ADDR, OUTX_L_M, raw_x, raw_y, raw_z);

    // Set the Variables
    Vector3 data;
    data.x = convertMag(raw_x);
    data.y = convertMag(raw_y);
    data.z = convertMag(raw_z);
    return data;
}

/* Main */
/*
int main_i2C() {

    // Initialize the I2C Data
    uBit.serial.printf("Initializing LSM6DS3TR-C & LIS3MDL...\r\n");
    initIMU();
    initMagnetometer();
    uBit.serial.printf("IMU & Magnetometer initialized.\r\n");

    // Basic
    Vector3 Accel;
    Vector3 Gyro;
    Vector3 Mag;
    bool firstIteration = true;

    // While Loop
    while (true) {
        // Get the Data
        Accel = getAccel();
        Gyro = getGyro();
        Mag = getMag();

        // Convert to Printable Format
        char axBuffer[12], ayBuffer[12], azBuffer[12];
        char mxBuffer[12], myBuffer[12], mzBuffer[12];

        floatToChar(Accel.x, axBuffer, ROUND_STEPS);
        floatToChar(Accel.y, ayBuffer, ROUND_STEPS);
        floatToChar(Accel.z, azBuffer, ROUND_STEPS);
        floatToChar(Mag.x, mxBuffer, ROUND_STEPS);
        floatToChar(Mag.y, myBuffer, ROUND_STEPS);
        floatToChar(Mag.z, mzBuffer, ROUND_STEPS);

        // Serial Print the Result // TODO: Create a more efficient method for printing on lines
        if (!firstIteration) moveCursorUp(9);
        firstIteration = false;

        uBit.serial.printf(
            "\rACCEL X (g): %s          \r\n"
            "ACCEL Y (g): %s          \r\n"
            "ACCEL Z (g): %s          \r\n"

            "GYRO X (dps): %d          \r\n"
            "GYRO Y (dps): %d          \r\n"
            "GYRO Z (dps): %d          \r\n"

            "MAG X (uT): %s          \r\n"
            "MAG Y (uT): %s          \r\n"
            "MAG Z (uT): %s          \r\n",

            axBuffer, ayBuffer, azBuffer,
            roundToInt(Gyro.x), roundToInt(Gyro.y), roundToInt(Gyro.z),
            mxBuffer, myBuffer, mzBuffer);

        uBit.sleep(50);
    }
}


//// SHARED
int main() {

    uBit.init();

    main_i2C();
    //main_motor_control();
}
*/


/* //// NOTES
What I want the robot to do: 
    #1 Know its relative position and data to its Calibration point

    #2 Know the best speed setting for the wheels to calculate a straight-line path
*/


//// ARCHIVE
/*
#include "samples/Tests.h"
    // Content: out_of_box_experience();
    Image:
        const char * const sun =
        "255,000,255,000,255\n"
        "000,255,255,255,000\n"
        "255,255,255,255,255\n"
        "000,255,255,255,000\n"
        "255,000,255,000,255\n";
        static const MicroBitImage SUN(sun);

    Buttons:
        static void onButtonA(MicroBitEvent) {
            DMESG("Button A");
            uBit.display.print(SUN);
        }
        static void onButtonB(MicroBitEvent) {
            DMESG("Button B");
            uBit.display.print(MOON);
    }
        main() {
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, onButtonB);
        }
Printing
    int i = 0;
    uBit.serial.printf("A Number %d\n", i);
*/