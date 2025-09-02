#include "MicroBit.h"
#include "helpers.h"
#include "Tests.h"
#include "imuFusion.h"

#include "pidServo.h"

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include <cmath>
#include <cstdint>

//Microbit
MicroBit uBit;

int main()
{
    uBit.init(); //leaving in because also calling uBit.display
    scheduler_init(uBit.messageBus);
    fiber_scheduler();

    while(1){
        fiber_sleep(1000);
    }
}
