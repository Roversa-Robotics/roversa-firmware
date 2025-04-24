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
int menu_first_enter=0;

static void scrollFromBottom(MicroBitImage image){
    for (int y=4; y >= 0; y--){
        uBit.display.image.paste(image,0,y);
        uBit.sleep(100);
    }
}
static void scrollFromTop(MicroBitImage image){
    for (int y=0; y <= 4; y++){
        uBit.display.image.paste(image,0,y-4);
        uBit.sleep(100);
    }
}

static void handleMenu(){
    MicroBitImage img;
    switch(menu_option){
        case 0:
            img = person_img;
            break;
        case 1:
            img = motor_img;
            break;
        case 2: //dist
            img = distance_img;
            break;
        case 3: //turn deg
            img = turn_img;
            break;
        case 4: //volume
            img = volume_img;
            break; 
    }
    uBit.display.clear();
    if(menu_previous==menu_option || menu_first_enter==1){
        uBit.display.image.paste(img,0,0);
        menu_first_enter = 0;
    }
    else if(menu_previous<menu_option){ //pressed down, scroll from bottom to top
        scrollFromBottom(img);
    }
    else { //pressed up, scroll from top
        scrollFromTop(img);
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
        menu_previous = menu_option;
        if(menu_option>0){
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
        menu_previous = menu_option;
        if(menu_option<4){
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

static void handleSubmenu(){
    uBit.display.clear();
    uBit.display.print("*");

    //Put in specific logic under each case
    // switch(menu_option){
    //     case 0:
    //     case 1:
    //     case 2:
    //     case 3:
    //     case 4:
    // }
}


static void menuHandler(MicroBitEvent){
    menu_press+=1;
    if(menu_press<2){
        return;
    }
    if(menu_press%2==0){
        menu_first_enter=1;
        handleMenu();
    }
    else{ //entered submenu, always an odd # press
        handleSubmenu();
    }
    
}

static void playAll(MicroBitEvent){
    MicroBitImage display_img;
    unsigned long time;
    while(*actionsCopy!='\0' && pause_flag==0){ //only play actions if queue not empty, not paused
        switch(*actionsCopy) {
            case 'F':
                display_img = forward_arrow;
                time = DRIVE_TIME;
                break;
            case 'B':
                display_img =reverse_arrow;
                time = DRIVE_TIME;
                break;
            case 'L':
                display_img =left_arrow;
                time = TURN_TIME;
                break;
            case 'R':
                display_img = right_arrow;
                time = TURN_TIME;
                break;
        }
        uBit.display.print(display_img);
        forward(100,100);
        fiber_sleep(time);
        stop();
        uBit.display.clear();

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
    if(menu_press>=2){
        menu_press = 0;
        menu_option = 0;
        menu_previous = 0;
        uBit.display.clear();
    }
    else{
        stop();
        for(int i=0;i<num_actions;i++){
            actions[i] = '\0'; //no actions in playAll will match
        }
        num_actions = 0;
        pause_flag=-1;
    }
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