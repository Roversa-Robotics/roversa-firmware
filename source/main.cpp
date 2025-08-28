#include "MicroBit.h"
#include "helpers.h"

#include "imuFusion.h"

#include "pidServo.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>

//Microbit
MicroBit uBit;

// Constants
const unsigned int new_SAMPLE_RATE = 100; // 100 Samples per Second
const float DELTA = 1.0f/new_SAMPLE_RATE; // In Seconds (0.01 s currently)

int main()
{
    // pidServo
    float MAX_DRIVE_TIME = 2000.f; // In msec
    float start_drive_time; // In msec

    //// Init
    // UBit Setup
    uBit.init();
    scheduler_init(uBit.messageBus);
    fiber_scheduler();

    // IMU Fusion
    initIMUFusion(new_SAMPLE_RATE);

    while(1){
        // Update Fusion
        updateIMUFusion();

        // Update PIDServo
        do_print = true
        updatePIDServo(do_print);

        // Wait
        uBit.sleep(DELTA * 1000);
    }
}

