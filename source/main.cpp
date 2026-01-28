#include "MicroBit.h"
#include "Tests.h"

MicroBit uBit;

int main()
{
    uBit.init(); //leaving in because also calling uBit.display
    // listen for button A to start printing to serial
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, [](MicroBitEvent) {
        uBit.serial.printf("Button A clicked.\n");
    });

    scheduler_init(uBit.messageBus);
    fiber_scheduler();

    while(1){
        fiber_sleep(1000);
    }
}

