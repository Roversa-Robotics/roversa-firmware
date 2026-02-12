#include "MicroBit.h"
#include "Tests.h"
#include "imuFusion.h"
#include "Images.h"
#include <cstdint>

MicroBit uBit;

bool do_print = false;
// Constants imuFusion
const unsigned int new_SAMPLE_RATE = 100; // 100 Samples per Second
const float DELTA = 1.0f/new_SAMPLE_RATE; // In Seconds (0.01 s currently)
const unsigned int IMU_UPDATE_MS = 10;
const unsigned int PRINT_INTERVAL_MS = 200;


int main()
{
    
    uBit.init(); //leaving in because also calling uBit.display
    uBit.display.print(happy_img);
    uBit.serial.printf("Program Started\n");
    
    // Allow I2C and sensors to stabilize
    uBit.sleep(500);
    
    // Scan I2C bus to find devices
    scanI2CBus();
    
    // listen for button B to start printing to serial
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, [](MicroBitEvent) {
        uBit.serial.printf("Button B clicked.\n");
        do_print = !do_print;
    });
    // IMU Fusion
    initIMUFusion(new_SAMPLE_RATE);

    scheduler_init(uBit.messageBus);
    fiber_scheduler();

    unsigned long last_print_time = 0;
    while(1){
        fiber_sleep(IMU_UPDATE_MS);
        //// Update Fusion
        updateIMUFusion();

        if (do_print) {
            unsigned long now = uBit.systemTime();
            if ((now - last_print_time) >= PRINT_INTERVAL_MS) {
                moveCursorDown(2);
                uBit.serial.printf("Fusion Printout:");
                moveCursorDown(1);
                printIMUFusion();
                last_print_time = now;
            }
        }
    }
}
