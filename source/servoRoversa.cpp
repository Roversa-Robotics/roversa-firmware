#include "MicroBit.h"
#include "servoRoversa.h"

extern MicroBit uBit;

/*
Feetech FS90R servo motor parameters
Range: 700μs - 2300μs, neutral at 1500μs, and a frequency of 50Hz
Left motor on microbit pin 1
Right motor on microbit pin 2
*/

const int period = 1000000 / FREQUENCY;

const int max_input = 100;                                                       //can be changed, default percentage: 100 is 100% CCW
const int min_input = -100;                                                      //can be changed, default percentages: -100 is 100% CW

long mapRange(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
drive function allows for full control of left and right motors full 
speed in both directions. The defaults can be changed by adjusting max_input
and min_input. 

Examples:
drive(0,0); neutral
drive(100,100); //forward
drive(-100,-100); //reverse
drive(-100,100); //left turn
drive(100,-100); //right turn
*/
void drive(int speedL, int speedR) {                                              //-100-100 each function will take care of wheel direction - 0 is neutral
    int angleSpeedL = mapRange(speedL,min_input,max_input,0,180);                 //user decides direction of motor L motor is CCW for forward
    int angleSpeedR = mapRange(speedR,min_input,max_input,180,0);                 //user decides direction of motor R motor is CW for forward
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(angleSpeedL, MAX_US, STOP_US);                       //left motor
    uBit.io.P2.setServoValue(angleSpeedR, MAX_US, STOP_US);                       //right motor
}

/*
forward, reverse, left, and right are set so you only have to use the midpoint
of the range for the max_input and min_input. The defaults allow 0-100% in left
and right motor and assumes direction. User only needs to put in speed value.
*/
void forward(int speedL, int speedR) {                                            //0-100 values with directions preset i.e. CCW, CW for each side
    int angleSpeedL = mapRange(speedL,(max_input+min_input),max_input,90,180);    //left motor CCW
    int angleSpeedR = mapRange(speedR,(max_input+min_input),max_input,90,0);      //right motor CW
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(angleSpeedL, MAX_US, STOP_US);
    uBit.io.P2.setServoValue(angleSpeedR, MAX_US, STOP_US);
}

void reverse(int speedL, int speedR) {
    int angleSpeedL = mapRange(speedL,(max_input+min_input),max_input,90,0);      //left motor CW
    int angleSpeedR = mapRange(speedR,(max_input+min_input),max_input,90,180);    //right motor CCW
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(angleSpeedL, MAX_US, STOP_US);       
    uBit.io.P2.setServoValue(angleSpeedR, MAX_US, STOP_US);     
}

void left(int speedL, int speedR) {
    int angleSpeedL = mapRange(speedL,(max_input+min_input),max_input,90,0);      //left motor CW
    int angleSpeedR = mapRange(speedR,(max_input+min_input),max_input,90,0);      //right motor CW
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(angleSpeedL, MAX_US, STOP_US);       
    uBit.io.P2.setServoValue(angleSpeedR, MAX_US, STOP_US); 
}

void right(int speedL, int speedR){
    int angleSpeedL = mapRange(speedL,(max_input+min_input),max_input,90,180);    //left motor CCW
    int angleSpeedR = mapRange(speedR,(max_input+min_input),max_input,90,180);    //right motor CCW
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(angleSpeedL, MAX_US, STOP_US);       
    uBit.io.P2.setServoValue(angleSpeedR, MAX_US, STOP_US); 
}

//set GPIO low to cut motors off - reset analog setServoPulseUs
void stop() {
    uBit.io.P1.setDigitalValue(0);                                                //left motor GPIO low - no signal to motor
    uBit.io.P2.setDigitalValue(0);                                                //right motor GPIO low - no signal to motor
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
}

/*
neutral position of any input range, if properly trimmed motor 
should be stopped
*/
void neutral() {
    uBit.io.P1.setServoPulseUs(period);                                           //set left motor period for analog
    uBit.io.P2.setServoPulseUs(period);                                           //set right motor period for analog
    uBit.io.P1.setServoValue(0, MAX_US, STOP_US);       
    uBit.io.P2.setServoValue(0, MAX_US, STOP_US); 
}
