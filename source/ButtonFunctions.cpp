#include "MicroBit.h"
#include "Tests.h"
#include "servoRoversa.h"

#define DRIVE_TIME 1350
#define TURN_TIME 650

int num_actions = 0;
char* actions = (char*)malloc(50*sizeof(char)); //hold up to 50 actions

void updateQueue(int pin){
    if(pin==MICROBIT_ID_IO_P13){
        actions[num_actions]='F';
    }
    else if(pin==MICROBIT_ID_IO_P14){
        actions[num_actions]='B'; //B for back/reverse
    }
    else if(pin==MICROBIT_ID_IO_P16){
        actions[num_actions]='L';
    }
    else if(pin==MICROBIT_ID_IO_P15){
        actions[num_actions]='R'; 
    }
    num_actions+=1;
}

static void addForward(MicroBitEvent){
    updateQueue(MICROBIT_ID_IO_P13);
}
static void addReverse(MicroBitEvent){
    updateQueue(MICROBIT_ID_IO_P14);
}
static void addLeft(MicroBitEvent){
    updateQueue(MICROBIT_ID_IO_P16);
}
static void addRight(MicroBitEvent){
    updateQueue(MICROBIT_ID_IO_P15);
}

static void printQueue(MicroBitEvent){
    for(int i=0;i<num_actions;i++){
        uBit.serial.printf("action: %c\n",actions[i]);
    }
}

//Button images:
MicroBitImage forward_arrow("0,0,255,0,0\n0,255,255,255,0\n255,0,255,0,255\n0,0,255,0,0\n0,0,255,0,0\n");
MicroBitImage reverse_arrow("0,0,255,0,0\n0,0,255,0,0\n255,0,255,0,255\n0,255,255,255,0\n0,0,255,0,0\n");
MicroBitImage left_arrow("0,0,255,0,0\n0,255,0,0,0\n255,255,255,255,255\n0,255,0,0,0\n0,0,255,0,0\n");
MicroBitImage right_arrow("0,0,255,0,0\n0,255,255,255,0\n255,0,255,0,255\n0,0,255,0,0\n0,0,255,0,0\n");

int stop_flag=0;

static void playAll(MicroBitEvent){
    for(int i=0;i<num_actions;i++){
        if(stop_flag == 1){
            uBit.serial.printf("full stop\n");
            break;
        }
        else if(actions[i]=='F'){
            uBit.display.print(forward_arrow);
            forward(100,100);
            uBit.sleep(DRIVE_TIME); //do nothing/allow to drive for DRIVE_TIME
            stop();
            uBit.display.clear();
        }
        else if(actions[i]=='B'){
            uBit.display.print(reverse_arrow);
            reverse(100,100);
            uBit.sleep(DRIVE_TIME);
            stop();
            uBit.display.clear();
        }
        else if(actions[i]=='L'){
            uBit.display.print(left_arrow);
            left(100,100);
            uBit.sleep(TURN_TIME);
            stop();
            uBit.display.clear();
        }
        else if(actions[i]=='R'){
            uBit.display.print(right_arrow);
            right(100,100);
            uBit.sleep(TURN_TIME);
            stop();
            uBit.display.clear();
        }
        uBit.sleep(1000);//take a break between actions
    }
    free(actions); //done executing all actions
}

static void stop(MicroBitEvent){
    stop_flag = 1;
}

void listen_direction(){ //synchronous event handling
    uBit.messageBus.listen(MICROBIT_ID_IO_P13, MICROBIT_BUTTON_EVT_CLICK, addForward);
    uBit.messageBus.listen(MICROBIT_ID_IO_P14, MICROBIT_BUTTON_EVT_CLICK, addReverse);
    uBit.messageBus.listen(MICROBIT_ID_IO_P16, MICROBIT_BUTTON_EVT_CLICK, addLeft);
    uBit.messageBus.listen(MICROBIT_ID_IO_P15, MICROBIT_BUTTON_EVT_CLICK, addRight);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playAll);
    // uBit.messageBus.listen(MICROBIT_ID_IO_P9, MICROBIT_BUTTON_EVT_CLICK, stop); //504 error if try Stop button during Play (no scheduler, synchronous)
    uBit.messageBus.listen(MICROBIT_ID_IO_P8, MICROBIT_BUTTON_EVT_CLICK, printQueue);
    while(1){
        uBit.sleep(1000);
    }
}

void fiber_scheduler(){ //asynchronous event handling
    scheduler_init(uBit.messageBus);
    uBit.messageBus.listen(MICROBIT_ID_IO_P13, MICROBIT_BUTTON_EVT_CLICK, addForward);
    uBit.messageBus.listen(MICROBIT_ID_IO_P14, MICROBIT_BUTTON_EVT_CLICK, addReverse);
    uBit.messageBus.listen(MICROBIT_ID_IO_P16, MICROBIT_BUTTON_EVT_CLICK, addLeft);
    uBit.messageBus.listen(MICROBIT_ID_IO_P15, MICROBIT_BUTTON_EVT_CLICK, addRight);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playAll);
    uBit.messageBus.listen(MICROBIT_ID_IO_P9, MICROBIT_BUTTON_EVT_CLICK, stop); //can interrupt playAll
    uBit.messageBus.listen(MICROBIT_ID_IO_P8, MICROBIT_BUTTON_EVT_CLICK, printQueue); //menu prints queue
    
    while(1){
        fiber_sleep(1000);
    }
}
