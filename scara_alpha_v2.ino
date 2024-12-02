/*
@file: scara_alpha_v1_0.ino
@updated: 2024/12/01
@author: Kensei Suzuki & Diego Gomez
@brief: Alpha version 2.0 meant for development. Top level main code for controlling the SCARA robot. 
*/

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  LIBRARIES | GLOBALS | OBJECTS | FUNCTION PROTOTYPES
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution

// pins
const int BASE_DIR_PIN        = 12;        //nema stepper motors control pins
const int BASE_STEP_PIN       = 13;
const int Z_DIR_PIN           = 8;
const int Z_STEP_PIN          = 9;
const int ELBOW_DIR_PIN       = 10;
const int ELBOW_STEP_PIN      = 11;
const int TOP_LIMIT_PIN       = 4;        //limit switch 
const int BOTTOM_LIMIT_PIN    = 3;
const int LEFT_LIMIT_PIN      = 5;
const int RIGHT_LIMIT_PIN     = 6;
const int GRIPPER_PIN         = 7;       //gripper servo motor PWM pin

// globals
int base_max_step=0, base_min_step=0, z_max_step=0, z_min_step=0, elbow_min_step=0, elbow_max_step=0;
int user_inputs[3] = {0,0,0};

//class objects
AccelStepper BaseStepper(AccelStepper::DRIVER, BASE_STEP_PIN, BASE_DIR_PIN);
AccelStepper ZStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper ElbowStepper(AccelStepper::DRIVER, ELBOW_STEP_PIN, ELBOW_DIR_PIN);
Servo GripperServo;

//func protos
void calibrateLimits();
void zeroHome();
void moveAxis(AccelStepper, int, int);
int getManualInput(const char*, int, int);
int calculateStep(int, int, int, int, int);

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *          SETUP - COMMS, ACTUATORS, SENSORS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void setup(){
    //init comms
    Serial.begin(9600);
    while(!Serial){}

    //init limit switches
    pinMode(TOP_LIMIT_PIN, INPUT_PULLUP);
    pinMode(BOTTOM_LIMIT_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);

    //setup stepper motors
    BaseStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    BaseStepper.setAcceleration(NEMA17_MAX_ACCEL);
    BaseStepper.setSpeed(0);
    ZStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    ZStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ZStepper.setSpeed(0);
    ElbowStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ElbowStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    ElbowStepper.setSpeed(0);

    delay(250);
    Serial.println("SCARA is initialized.");

    calibrateLimits();
    zeroHome();
} //end of setup()

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop(){
    //get user input for each joint 
    int base_position = getManualInput("Enter base angle (+/- 90deg): ", -90, 90);
    int z_position = getManualInput("Enter z-axis height (192mm, 387mm): ", 192, 387);
    int elbow_position = getManualInput("Enter elbow angle (+/- 148deg): ", -148, 148);

    //convert user input into step positions for motors
    int b_step = calculateStep(base_position, -90, 90, base_min_step, base_max_step);
    int z_step = calculateStep(z_position, 192, 387, z_min_step, z_max_step);
    int e_step = calculateStep(elbow_position, -148, 148, elbow_min_step, elbow_max_step);

    //run motors to step positions
    Serial.print("Moving SCARA to position... ");
    moveAxis(BaseStepper, b_step);
    moveAxis(ZStepper, z_step);
    moveAxis(ElbowStepper, e_step);
    Serial.println("Done");

    delay(100);
} //end of loop()


/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                USER-DEFINED FUNCTIONS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

void calibrateLimits(){
    Serial.print("Calibrating joint limits ... ");
    
    // BaseStepper.setSpeed(-1 * NEMA17_MAX_SPEED / 10);
    // while( digitalRead(LEFT_LIMIT_PIN)==LOW ){
    //     BaseStepper.runSpeed();
    // }
    // base_min_step = BaseStepper.currentPosition();

    // BaseStepper.setSpeed(1 * NEMA17_MAX_SPEED / 10);
    // while( digitalRead(RIGHT_LIMIT_PIN)==LOW ){
    //     BaseStepper.runSpeed();
    // }
    // base_max_step = BaseStepper.currentPosition();

    //top
    ZStepper.setSpeed(NEMA17_MAX_SPEED / 3);
    while( digitalRead(TOP_LIMIT_PIN)==LOW ){
        ZStepper.runSpeed();
    }
    z_max_step = ZStepper.currentPosition();

    //bottom
    ZStepper.setSpeed(-1 * NEMA17_MAX_SPEED / 3);
    while( digitalRead(BOTTOM_LIMIT_PIN)==LOW ){
        ZStepper.runSpeed();
    }
    z_min_step = ZStepper.currentPosition();
        
    //left
    ElbowStepper.setSpeed(-1 * NEMA17_MAX_SPEED / 15);
    while( digitalRead(LEFT_LIMIT_PIN)==LOW ){
        ElbowStepper.runSpeed();
    }
    elbow_min_step = ElbowStepper.currentPosition();

    //right
    ElbowStepper.setSpeed(1 * NEMA17_MAX_SPEED / 15);
    while( digitalRead(RIGHT_LIMIT_PIN)==LOW ){
        ElbowStepper.runSpeed();
    }
    elbow_max_step = ElbowStepper.currentPosition();

    Serial.println("Done.");

} //end of calibrateLimits()

void zeroHome(){
    //move to center positions
    ZStepper.moveTo( floor(average(z_max_step, z_min_step)) );
    ZStepper.setSpeed(NEMA17_MAX_SPEED/3);
    ZStepper.run();
    ElbowStepper.moveTo( floor(average(elbow_max_step, elbow_min_step)) );
    ElbowStepper.setSpeed(NEMA17_MAX_SPEED/15);
    ElbowStepper.run();
} //end of zeroHome()

void moveAxis(AccelStepper joint, int stp, int speed_reduction){
    int d = 1;
    joint->setSpeed(d * NEMA17_MAX_SPEED / speed_reduction);
    joint->moveTo(stp);
    while(joint->distanceToGo() != 0){
        joint->run();
    }
}

int getManualInput(const char *prompt, int minVal, int maxVal){
    int input;

    do{
        Serial.print(prompt);
        while(!Serial.available()){}
        input = Serial.parseInt();
        if( input < minVal || input > maxVal ){
            Serial.println("Invalid input. Please try again.");
        }
    } while( input < minVal || input > maxVal);
    
    return input; 
}

int calculateStep(int input, int minInput, int maxInput, int minStep, int maxStep){
    return map(input, minInput, maxInput, minStep, maxStep); 
}