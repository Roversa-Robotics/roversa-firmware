#include "MicroBit.h"
#include "helpers.h"

#include "imuFusion.h"

#include "pidServo.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>


/*
@author: Rowland Halsey Robinson, 6/13/25
tbx9hk@virginia.edu
*/


//// Microbit
MicroBit uBit;
bool buttonAJustPressed = false;
int buttonAState = 0;
bool buttonBJustPressed = false;
int buttonBState = 0;

//// Main
// Constants
const unsigned int new_SAMPLE_RATE = 100; // 100 Samples per Second
const float DELTA = 1.0f/new_SAMPLE_RATE; // In Seconds (0.01 s currently)

// Buttons
static void onButtonA(MicroBitEvent) {
    buttonAJustPressed = true;
    buttonAState = (buttonAState + 1)%2;
}

static void onButtonB(MicroBitEvent) {
    buttonBJustPressed = true;
    buttonBState = (buttonBState + 1)%3;
}

// Main
int main() {

    //// Constants
    // Timing
    const float PRINT_INTERVAL = 500.0f; // In msec
    float print_start_time = uBit.systemTime() + PRINT_INTERVAL; // In msec

    // pidServo
    float MAX_DRIVE_TIME = 2000.f; // In msec
    float start_drive_time; // In msec

    //// Init
    // UBit Setup
    uBit.init();
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, onButtonB);

    // IMU Fusion
    initIMUFusion(new_SAMPLE_RATE);

    //// Main Loop
    while (true) {

        // adjust p i d values in realtime
        if (uBit.serial.isReadable()) {
            ManagedString input = uBit.serial.readUntil(ManagedString('\n'));
            char select = input.toCharArray()[0];
            float value = atof(input.toCharArray() + 1);
            if (select == 'p') { Kp = value; }
            else if (select == 'd') { Kd = value; }
            else if (select == 'i') { Ki = value; }
            else if (select == 's') {
                uBit.serial.printf("Kp: "); 
                printFloat(Kp);
                uBit.serial.printf("Ki: ");
                printFloat(Ki);
                uBit.serial.printf("Kd: ");
                printFloat(Kd);
            }
        }   

        //// Printing
        bool do_print = false;

        // Check Print
        float print_end_time = uBit.systemTime();
        if ((print_end_time - print_start_time) >= PRINT_INTERVAL) {
            do_print = true;
            print_start_time = print_end_time;
        }

        //// Update Fusion
        updateIMUFusion();

        //// Triggers
        // Button A Pressed
        if (buttonAJustPressed) {
            do_print = true;

            // Idle State
            if (buttonAState == 0) {}

            // Printout the Fusion
            if (buttonAState == 1) {}
        }

        // Time Elapsed
        else if (is_driving() & ((uBit.systemTime() - start_drive_time) >= MAX_DRIVE_TIME)) {
            buttonBJustPressed = true;
            buttonBState = 2;
        }

        // Button B Pressed
        if (buttonBJustPressed) {
            do_print = true;

            // Idle State
            if (buttonBState == 0) {}

            // Make the Bot Drive
            if (buttonBState == 1) {
                start_drive_time = uBit.systemTime();
                forward();
            }

            // Stop the Bot
            if (buttonBState == 2) {
                stop();
            }
        }

        //// Do Print
        if (do_print & (buttonBState != 2)) {

            // Reset Screen
            moveCursorUp(30);

            // Informational
            moveCursorDown(1);
            uBit.serial.printf("Setup Complete:");
            moveCursorDown(1);
            uBit.serial.printf("Press A to Toggle the Fusion Printout");
            moveCursorDown(1);
            uBit.serial.printf("Press B to Toggle the PID Loop");
            moveCursorDown(1);
            uBit.serial.printf("After PID Loop Runs, Output will hold until B button is again pressed");
            moveCursorDown(1);

            // Fusion
            if (buttonAState == 1) {
                moveCursorDown(2);
                uBit.serial.printf("Fusion Printout:");
                moveCursorDown(1);
                printIMUFusion();
                uBit.serial.printf("yaw: ");
                printFloat(getYaw());
            }

            // pidServo
            if (buttonBState == 1) {
                moveCursorDown(2);
                uBit.serial.printf("PID Loop:");
                moveCursorDown(1);
                // Printout for pidServo happens below
            }
        }

        //// Update pidServo
        updatePIDServo(do_print);

        // Thought was Driving, but is no longer Driving
        if (!is_driving() & (buttonBState == 1)) {
            buttonBState = 2;
        }

        //// Reset the Buttons being Pressed
        buttonAJustPressed = false;
        buttonBJustPressed = false;

        //// Wait
        uBit.sleep(DELTA * 1000);
    }
}