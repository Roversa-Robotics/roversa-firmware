#include "MicroBit.h"
#include "Tests.h"
#include "servoRoversa.h"
#include "Buttons.h"
#include "Images.h"
#include "imuFusion.h"

#include <cmath>

#define DRIVE_TIME 1350
#define TURN_TIME 650
#define ACTIONS_LIMIT 50 //hold up to 50 actions

static const int BASE_FORWARD_SPEED = 80;
static const int MAX_SPEED = 100;
static const int MIN_SPEED = 0;
static const int MAX_CORRECTION = 30;
static const float PID_KP = 1.0f;
static const float PID_KI = 0.0f;
static const float PID_KD = 0.1f;
static const float INTEGRAL_LIMIT = 100.0f;
static const float YAW_CORRECTION_SIGN = -1.0f;

/*  
    * actions like a queue of commands: each char is either forward ('F'), reverse ('B'), turn left ('L'), or turn right ('R').
    * actions_copy copies the queue for iterating through when commands played
    * actions populated by updateQueue() when F/B/L/R arrows pressed
    * actions cleared by stopHandler() 
*/
int num_actions = 0;
char * actions = (char*)malloc(ACTIONS_LIMIT*sizeof(char));
char * actions_copy;


/* track the state of the program; pause_flag of
    * -1 = program is not in play
        (either play button not pressed at all yet, or all commands in queue were executed so flag restored to -1)
    * 0 = play program 
        (actions are executing in playActions())
    * 1 = pause program 
        (play button pressed again, pausing execution in playActions())
 */
int pause_flag=-1;

static float pid_target_yaw = 0.0f;
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;


/* menu tracking
    * menu_press>=2 means user is "in" the menu
    * menu_option represents which calibration option user currently sees on screen/working with
    * menu_option_prev is calibration option seen just before current menu_option, used for scroll effect management in mainMenuDisplay
*/
int menu_press=0;
int menu_option_prev=0;
int menu_option=0;
int entering_main=false;
int in_main=false;


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
static void mainMenuDisplay(){
    uBit.display.setBrightness(255);
    MicroBitImage img;
    switch(menu_option){
        case 0: // lang
            img = person_img;
            break;
        case 1: // motors
            img = motor_img;
            break;
        case 2: // dist
            img = distance_img;
            break;
        case 3: // turn deg
            img = turn_img;
            break;
        case 4: // volume
            img = volume_img;
            break; 
    }
    uBit.display.clear();
    if(menu_option_prev==menu_option || entering_main){ // for first, last menu_options (case 0, case 4), menu_option_prev==menu_option; paste image without scroll-in effects
        uBit.display.image.paste(img,0,0);
        entering_main = false;
    }
    else if(menu_option_prev<menu_option){ // go to option below; scroll image in bottom to top
        scrollFromBottom(img);
    }
    else { // pressed up, scroll image in top to bottom
        scrollFromTop(img);
    }
}

static void submenuDisplay(){
    for(int i=0;i<2;i++){ // flashing symbol: entered submenu for current menu_option
        uBit.display.setBrightness(75);
        uBit.sleep(400);
        uBit.display.setBrightness(255);
        uBit.sleep(400);
    }
    uBit.display.setBrightness(75);
}

static int clampInt(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float normalizeAngle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void resetHeadingHold() {
    pid_target_yaw = getYaw();
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
}

static void applyHeadingHold(int base_speed) {
    float current_yaw = getYaw();
    float error = normalizeAngle(pid_target_yaw - current_yaw);
    float dt = getDelta();
    if (dt <= 0.0f) {
        dt = 0.01f;
    }

    pid_integral += error * dt;
    if (pid_integral > INTEGRAL_LIMIT) {
        pid_integral = INTEGRAL_LIMIT;
    } else if (pid_integral < -INTEGRAL_LIMIT) {
        pid_integral = -INTEGRAL_LIMIT;
    }

    float derivative = (error - pid_prev_error) / dt;
    pid_prev_error = error;

    float output = (PID_KP * error) + (PID_KI * pid_integral) + (PID_KD * derivative);
    output *= YAW_CORRECTION_SIGN; // Flip if corrections steer the wrong direction.

    if (output > MAX_CORRECTION) {
        output = MAX_CORRECTION;
    } else if (output < -MAX_CORRECTION) {
        output = -MAX_CORRECTION;
    }

    int correction = static_cast<int>(std::lround(output));
    int left_speed = clampInt(base_speed + correction, MIN_SPEED, MAX_SPEED);
    int right_speed = clampInt(base_speed - correction, MIN_SPEED, MAX_SPEED);
    forward(left_speed, right_speed);
}

static void driveForwardWithHeadingHold(unsigned long duration_ms) {
    
    unsigned long start_time = uBit.systemTime();
    while ((uBit.systemTime() - start_time) < duration_ms && pause_flag == 0) {
        applyHeadingHold(BASE_FORWARD_SPEED);
        fiber_sleep(10);
    }
}

static void updateQueue(int pin){
    if(num_actions<ACTIONS_LIMIT && pause_flag==-1){
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

/* user presses forward arrow; if statements handle whether user wanting to 
    * scroll main menu
    * calibrate a menu_option in submenu
    * add forward move command 
*/
static void addForward(MicroBitEvent){
    if(menu_press>=2 && in_main){ // cycle thru main menu
        menu_option_prev = menu_option;
        if(menu_option>0){
            menu_option-=1;
        }
        mainMenuDisplay();
    }
    else if(menu_press>=2){ // handle forward button press based on current menu_option
        switch(menu_option){
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
        }
    }
    else{
        updateQueue(MICROBIT_ID_IO_P13);
    }
}

static void addReverse(MicroBitEvent){ // same logic as described for addForward
    if(menu_press>=2 && in_main){
        menu_option_prev = menu_option;
        if(menu_option<4){
            menu_option+=1;
        }
        mainMenuDisplay();  
    }
    else if(menu_press>=2){
        switch(menu_option){
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
        }
    }
    else{
        updateQueue(MICROBIT_ID_IO_P14);
    }
}

static void addLeft(MicroBitEvent){ // same logic as described for addForward
    if(menu_press>=2){
        switch(menu_option){
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
        }
    }
    else{
        updateQueue(MICROBIT_ID_IO_P16);
    }
}

static void addRight(MicroBitEvent){ // same logic as described for addForward
    if(menu_press>=2){
        switch(menu_option){
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
        }
    }
    else{
        updateQueue(MICROBIT_ID_IO_P15);
    }
}


static void menuHandler(MicroBitEvent){
    menu_press+=1;
    if(menu_press<2){
        return;
    }
    else if(menu_press%2==0){
        entering_main=true;
        in_main = true;
        mainMenuDisplay();
    }
    else{ // entered submenu; odd # of presses
        in_main = false;
        submenuDisplay();
    }   
}

static void playHandler(MicroBitEvent){
    if(pause_flag==1){
        pause_flag=0;
    }
    else{
        if(num_actions>0){
            if(pause_flag==-1){
                actions_copy = actions;
            }
            pause_flag+=1;
        }
    }
}

static void playActions(MicroBitEvent){
    MicroBitImage display_img;
    unsigned long time = 100;
    // I want to hold the heading after each turn,
    // so reset the PID controller's target and integral/derivative terms at the start of the play sequence.
    resetHeadingHold();
    while(*actions_copy!='\0' && pause_flag==0){ // only play actions if queue not empty, not paused
        switch(*actions_copy) {
            case 'F':
                display_img = forward_arrow;
                //time = DRIVE_TIME;
                time = 3000;
                break;
            case 'B':
                reverse(100,100);
                display_img =reverse_arrow;
                time = DRIVE_TIME;
                break;
            case 'L':
                left(100,100);
                display_img =left_arrow;
                time = TURN_TIME;
                resetHeadingHold();
                break;
            case 'R':
                right(100,100);
                display_img = right_arrow;
                time = TURN_TIME;
                resetHeadingHold();
                //print 
                uBit.serial.printf("Turned right, new target yaw: %.2f\n", pid_target_yaw);
                break;
        }
        // This is new. I would like to add it inside the case switch, but will leave it here for now 
        //to avoid any issues with the timing of the display vs motor commands.
        //  If it works well, can consider moving into each case in the future.
        uBit.display.print(display_img);
        if (*actions_copy == 'F') {
            driveForwardWithHeadingHold(time);
        } else {
            fiber_sleep(time);
        }
        stop();
        uBit.display.clear();
        actions_copy+=1; //iterate through commands
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


static void stopHandler(MicroBitEvent){
    if(menu_press>=2){ //reset and "exit" menu
        menu_press=0;
        menu_option_prev=0;
        menu_option=0;
        entering_main=false;
        in_main=false;
        uBit.display.clear();
    }
    else{ //clear program, reset flags
        stop();
        for(int i=0;i<num_actions;i++){
            actions[i] = '\0';
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
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playActions);
    uBit.messageBus.listen(MICROBIT_ID_IO_P5, MICROBIT_BUTTON_EVT_CLICK, playHandler);
    uBit.messageBus.listen(MICROBIT_ID_IO_P9, MICROBIT_BUTTON_EVT_CLICK, stopHandler, MESSAGE_BUS_LISTENER_IMMEDIATE);
    uBit.messageBus.listen(MICROBIT_ID_IO_P8, MICROBIT_BUTTON_EVT_CLICK, menuHandler);
}