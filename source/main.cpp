#include "MicroBit.h"

#include "servoRoversa.h"
#include "i2c.h"

#include "helpers.h"
#include "tests.h"

MicroBit uBit;

int test = 0;
int main()
{
    uBit.init();

    Vector3 v1(1,2,3);
    Vector3 v2(5,-7,11);
    testVector3(v1, v2, 2);

    while(true) {
        uBit.sleep(100);
    }
}

