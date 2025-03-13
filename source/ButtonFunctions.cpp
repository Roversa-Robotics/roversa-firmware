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
    if(num_actions<ACTIONS_LIMIT){
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


static void playAll(MicroBitEvent){
    while(*actions!='\0'){
        if(pause_flag==1||stop_flag==1){
            break;
        }
        else if(*actions=='F'){
            uBit.display.print(forward_arrow);
            forward(100,100);
            uBit.sleep(DRIVE_TIME); //do nothing, allow to drive for DRIVE_TIME
            stop();
            uBit.display.clear();
        }
        else if(*actions=='B'){
            uBit.display.print(reverse_arrow);
            reverse(100,100);
            uBit.sleep(DRIVE_TIME);
            stop();
            uBit.display.clear();
        }
        else if(*actions=='L'){
            uBit.display.print(left_arrow);
            left(100,100);
            uBit.sleep(TURN_TIME);
            stop();
            uBit.display.clear();
        }
        else if(*actions=='R'){
            uBit.display.print(right_arrow);
            right(100,100);
            uBit.sleep(TURN_TIME);
            stop();
            uBit.display.clear();
        }
        actions+=1; //iterate through commands
        num_actions-=1; //num remaining decremented
        uBit.sleep(1000);//take a break between actions
    }
    if(num_actions==0||stop_flag==1){ //exited loop because finished all actions, or terminated. Condition necessary so not resetting flag during an actual pause
        uBit.serial.printf("Resetting flag\n");
        pause_flag=-1;
        stop_flag=0;
    }
    uBit.serial.printf("Pause flag:%d\n",pause_flag);
    uBit.serial.printf("Stop flag:%d\n",stop_flag);
}

static void stopHandler(MicroBitEvent){
    if(pause_flag==0){
        stop_flag = 1; //means program stopped after running
    }
    for(int i=0;i<num_actions;i++){
        actions[i] = '\0'; //clear all actions in program
    }
    num_actions = 0; //clear program
    pause_flag=-1;
    uBit.serial.printf("Stop handler\n");
    uBit.serial.printf("Pause flag:%d\n",pause_flag);
    uBit.serial.printf("Stop flag:%d\n",stop_flag);
}

static void playHandler(MicroBitEvent){ //possibly call playHandler as a void() instead of another event --> ensure playHandler always before playAll, not concurrent
    if(pause_flag==1){
        pause_flag=0;
    }
    else{
        if(num_actions>0){ //don't let messing with play button on empty program mess with flag
            pause_flag+=1; //set flag
        }
    }
}

void fiber_scheduler(){ //asynchronous event handling
    scheduler_init(uBit.messageBus);
    uBit.messageBus.listen(MICROBIT_ID_IO_P13, MICROBIT_BUTTON_EVT_CLICK, addForward);
    uBit.messageBus.listen(MICROBIT_ID_IO_P14, MICROBIT_BUTTON_EVT_CLICK, addReverse);
    uBit.messageBus.listen(MICROBIT_ID_IO_P16, MICROBIT_BUTTON_EVT_CLICK, addLeft);
    uBit.messageBus.listen(MICROBIT_ID_IO_P15, MICROBIT_BUTTON_EVT_CLICK, addRight);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playAll);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playHandler);
    uBit.messageBus.listen(MICROBIT_ID_IO_P9, MICROBIT_BUTTON_EVT_CLICK, stopHandler); //can interrupt playAll
    uBit.messageBus.listen(MICROBIT_ID_IO_P8, MICROBIT_BUTTON_EVT_CLICK, printQueue); //menu prints queue
    
    while(1){
        fiber_sleep(1000);
    }
}
