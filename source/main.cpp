#include "MicroBit.h"
#include "Tests.h"

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

