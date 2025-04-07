#include "MicroBit.h"
#include "Tests.h"
#include "servoRoversa.h"
#include "Buttons.h"
#include "Images.h"

#define DRIVE_TIME 1350
#define TURN_TIME 650
#define ACTIONS_LIMIT 50 //hold up to 50 actions


int num_actions = 0;
char* actions = (char*)malloc(ACTIONS_LIMIT*sizeof(char));
int stop_flag=0;
int pause_flag=-1;

void updateQueue(int pin){
    uBit.serial.printf("updateQueue:\n");
    uBit.serial.printf("num_actions:%d\n",num_actions);
    uBit.serial.printf("pause_flag:%d\n",pause_flag);
    if(num_actions<ACTIONS_LIMIT && pause_flag==-1){ //can't add after start playing(0) or paused(1). If want to allow update during pause, change condition to !=0
        uBit.serial.printf("Check actions[] after prev program\n");
        uBit.serial.printf("actions[num_actions]: %s\n", actions[num_actions]);
        switch(pin) {
            case MICROBIT_ID_IO_P13:
                actions[num_actions]='F';
                break;
            case MICROBIT_ID_IO_P14:
                actions[num_actions]='B';
                break;
            case MICROBIT_ID_IO_P16:
                actions[num_actions]='L';
                break;
            case MICROBIT_ID_IO_P15:
                actions[num_actions]='R';
                break;
        }
        num_actions+=1;
    }
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
    if(pause_flag!=0){ //can view menu if before, after, or paused program
        if(*actions=='\0'){
            uBit.serial.printf("No actions in queue\n");
        }
        else{
            char *actionsCopy = actions;
            while(*actionsCopy!='\0'){
                uBit.serial.printf("%c\n",*actionsCopy);
                actionsCopy+=1;
            }
        }
    }
}

static void playAll(MicroBitEvent){
    while(*actions!='\0' && pause_flag==0){ //only play actions if queue not empty, not paused
        switch(*actions) {
            case 'F':
                uBit.display.print(forward_arrow);
                forward(100,100);
                fiber_sleep(DRIVE_TIME);
                stop();
                uBit.display.clear();
                break;
            case 'B':
                uBit.display.print(reverse_arrow);
                reverse(100,100);
                fiber_sleep(DRIVE_TIME);
                stop();
                uBit.display.clear();
                break;
            case 'L':
                uBit.display.print(left_arrow);
                left(100,100);
                fiber_sleep(TURN_TIME);
                stop();
                uBit.display.clear();
                break;
            case 'R':
                uBit.display.print(right_arrow);
                right(100,100);
                fiber_sleep(TURN_TIME);
                stop();
                uBit.display.clear();
                break;
        }
        actions+=1; //iterate through commands
        fiber_sleep(1000);
    }
    if(pause_flag==0){
        pause_flag=-1; // was in play (0), program just completed, reset flag for next program
        num_actions = 0;
    }
}

static void playHandler(MicroBitEvent){
    if(pause_flag==1){
        pause_flag=0;
    }
    else{
        if(num_actions>0){ //don't let play presses b/w empty queues mess with flag
            pause_flag+=1; //set flag if queue has actions to do
        }
    }
}

static void stopHandler(MicroBitEvent){ //clear program, reset flags (whether or not program running)
    stop();
    uBit.serial.printf("stop, num_actions: %d",num_actions);
    for(int i=0;i<num_actions;i++){
        actions[i] = 'c'; //no actions in playAll will match
    }
    num_actions = 0;
    pause_flag=-1;
}

void fiber_scheduler(){ //asynchronous event handling
    uBit.messageBus.listen(MICROBIT_ID_IO_P13, MICROBIT_BUTTON_EVT_CLICK, addForward);
    uBit.messageBus.listen(MICROBIT_ID_IO_P14, MICROBIT_BUTTON_EVT_CLICK, addReverse);
    uBit.messageBus.listen(MICROBIT_ID_IO_P16, MICROBIT_BUTTON_EVT_CLICK, addLeft);
    uBit.messageBus.listen(MICROBIT_ID_IO_P15, MICROBIT_BUTTON_EVT_CLICK, addRight);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playAll);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playHandler);
    uBit.messageBus.listen(MICROBIT_ID_IO_P9, MICROBIT_BUTTON_EVT_CLICK, stopHandler);
    uBit.messageBus.listen(MICROBIT_ID_IO_P8, MICROBIT_BUTTON_EVT_CLICK, printQueue);
}