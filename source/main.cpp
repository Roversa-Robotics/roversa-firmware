#include "MicroBit.h"
#include "Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();
    
    fiber_scheduler();

    // out_of_box_experience();

    microbit_panic( 999 );
}

