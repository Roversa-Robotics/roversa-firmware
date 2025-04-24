#include "MicroBit.h"
#include "Tests.h"
#include "servoRoversa.h"
#include "Buttons.h"
#include "Images.h"

#define DRIVE_TIME 1350
#define TURN_TIME 650
#define ACTIONS_LIMIT 50 //hold up to 50 actions


int num_actions = 0;
char * actions = (char*)malloc(ACTIONS_LIMIT*sizeof(char));
char * actionsCopy;
int stop_flag=0;
int pause_flag=-1;
int menu_press=0;
int menu_previous=0;
int menu_option=0;

static void handleMenu(){
    uBit.serial.printf("menu option #: %d\n",menu_option);
    switch(menu_option){
        case 0: //lang
            uBit.serial.printf("Displaying L\n");
            uBit.display.print("L"); 
            break;
        case 1: //motor calib
            uBit.serial.printf("Displaying M\n");
            uBit.display.clear();
            if(menu_previous<menu_option){
                for (int y=4; y >= 0; y--){
                    MicroBitImage motor("255,0,0,0,255\n255,255,0,255,255\n255,0,255,0,255\n255,0,0,0,255\n255,0,0,0,255\n");
                    uBit.display.image.paste(motor,0,y);
                    uBit.sleep(100);
                }
            }
            else{
                for (int y=0; y <= 4; y++){
                    MicroBitImage motor("255,0,0,0,255\n255,255,0,255,255\n255,0,255,0,255\n255,0,0,0,255\n255,0,0,0,255\n");
                    uBit.display.image.paste(motor,0,y-4);
                    uBit.sleep(100);
                }
            }
            
            break;
            // uBit.display.print("M"); 
        case 2: //dist
            uBit.serial.printf("Displaying D\n");
            uBit.display.clear();
            uBit.display.print("D"); 
            break;
        case 3: //turn deg
            uBit.serial.printf("Displaying T\n");
            uBit.display.clear();
            uBit.display.print("T");
            break;
        case 4: //volume
            uBit.serial.printf("Displaying V\n");
            uBit.display.clear();
            uBit.display.print("V");
            break;
    }
}

static void updateQueue(int pin){
    if(num_actions<ACTIONS_LIMIT && pause_flag==-1){ //can't add after start playing(0) or paused(1). If want to allow update during pause, change condition to !=0
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
    if(menu_press>=2){ //up arrow being used to cycle thru menu, not add F
        if(menu_option>0){
            menu_previous = menu_option;
            menu_option-=1;
        }
        handleMenu();
    }
    else{
        updateQueue(MICROBIT_ID_IO_P13);
    }
}
static void addReverse(MicroBitEvent){
    if(menu_press>=2){
        if(menu_option<4){
            menu_previous = menu_option;
            menu_option+=1;
        }
        handleMenu();  
    }
    else{
        updateQueue(MICROBIT_ID_IO_P14);
    }
}
static void addLeft(MicroBitEvent){
    updateQueue(MICROBIT_ID_IO_P16);
}
static void addRight(MicroBitEvent){
    updateQueue(MICROBIT_ID_IO_P15);
}

// static void printQueue(MicroBitEvent){
//     if(pause_flag!=0){ //can view menu if before, after, or paused program
//         if(*actions=='\0'){
//             uBit.serial.printf("No actions in queue\n");
//         }
//         else{
//             char *actionsCopy = actions; //prints full queue for now, not printing global actionsCopy in case print called before initialized with initial play
//             while(*actionsCopy!='\0'){
//                 uBit.serial.printf("%c\n",*actionsCopy);
//                 actionsCopy+=1;
//             }
//         }
//     }
// }
static void menuHandler(MicroBitEvent){
    //Enter the menu on some condition (or figure out condition later)
    //display icon based on number of presses since entered the menu
    menu_press+=1;
    if(menu_press<2){
        return;
    }
    //menu_press>=2; menu entered
    handleMenu();
}

static void playAll(MicroBitEvent){
    while(*actionsCopy!='\0' && pause_flag==0){ //only play actions if queue not empty, not paused
        switch(*actionsCopy) {
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
        actionsCopy+=1; //iterate through commands
        fiber_sleep(1000);
    }

    if(pause_flag==0){
        pause_flag=-1; // was in play (0), program just completed, reset flag for next program
        for(int i=0;i<num_actions;i++){
            actions[i] = '\0';
        }
        num_actions = 0;
    }
}

static void playHandler(MicroBitEvent){
    if(pause_flag==1){
        pause_flag=0;
    }
    else{
        if(num_actions>0){ //don't let play presses b/w empty queues mess with flag
            if(pause_flag==-1){
                actionsCopy = actions;
            }
            pause_flag+=1; //set flag if queue has actions to do
        }
    }
}

static void stopHandler(MicroBitEvent){ //clear program, reset flags (whether or not program running)
    stop();
    for(int i=0;i<num_actions;i++){
        actions[i] = '\0'; //no actions in playAll will match
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
    uBit.messageBus.listen(MICROBIT_ID_IO_P9, MICROBIT_BUTTON_EVT_CLICK, stopHandler, MESSAGE_BUS_LISTENER_IMMEDIATE);
    uBit.messageBus.listen(MICROBIT_ID_IO_P8, MICROBIT_BUTTON_EVT_CLICK, menuHandler);
}