#include "MicroBit.h"
#include "Tests.h"
#include "imuFusion.h"
#include "Images.h"

MicroBit uBit;

bool do_print = false;
// Constants imuFusion
const unsigned int new_SAMPLE_RATE = 100; // 100 Samples per Second
const float DELTA = 1.0f/new_SAMPLE_RATE; // In Seconds (0.01 s currently)


int main()
{
    
    uBit.init(); //leaving in because also calling uBit.display
    uBit.display.print(happy_img);
    uBit.serial.printf("Program Started\n");
    // listen for button B to start printing to serial
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, [](MicroBitEvent) {
        uBit.serial.printf("Button B clicked.\n");
        do_print = !do_print;
    });
    // IMU Fusion
    initIMUFusion(new_SAMPLE_RATE);

    scheduler_init(uBit.messageBus);
    fiber_scheduler();

    while(1){
        fiber_sleep(1000);
        //// Update Fusion
        updateIMUFusion();

        moveCursorDown(2);
        if (do_print) {
            uBit.serial.printf("Fusion Printout:");
            moveCursorDown(1);
            printIMUFusion();
        }
    }
}
