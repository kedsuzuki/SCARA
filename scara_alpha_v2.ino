/*
@file: scara_alpha_v2.ino
@updated: 2025/02/10
@author: Kensei Suzuki & Diego Gomez
@brief: Alpha version 2.0 meant for development. Top level main code for controlling the SCARA robot. 
        V2.0 includes a new naming convention as well as modularized and re-usable functions.

        Base --> J1
        Z-axis --> J2
        Elbow --> J3
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
#define MICROSTEP_FACTOR        8         //microstepping - increases steps/rev by factor of {2, 4, 8, 16}
 
// pins
const int J1_DIR_PIN        = 12;        //nema stepper motors control pins
const int J1_STEP_PIN       = 13;
const int J2_DIR_PIN           = 8;
const int J2_STEP_PIN          = 9;
const int J3_DIR_PIN       = 10;
const int J3_STEP_PIN      = 11;
const int J2_MAX_LIMIT_PIN       = 4;        //limit switch 
const int J2_MIN_LIMIT_PIN       = 3;
const int J3_MIN_LIMIT_PIN   = 5;
const int J3_MAX_LIMIT_PIN   = 6;
const int J1_LIMIT_PIN        = 2;
const int GRIPPER_PIN           = 7;       //gripper servo motor PWM pin

// globals
int J1_max_step=0, J1_min_step=0, J2_max_step=0, J2_min_step=0, J3_min_step=0, J3_max_step=0;
int user_inputs[3] = {0,0,0};

//class objects
AccelStepper J1Stepper(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper J2Stepper(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper J3Stepper(AccelStepper::DRIVER, J3_STEP_PIN, J3_DIR_PIN);
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
    pinMode(J2_MAX_LIMIT_PIN, INPUT_PULLUP);
    pinMode(J2_MIN_LIMIT_PIN, INPUT_PULLUP);
    pinMode(J3_MAX_LIMIT_PIN, INPUT_PULLUP);
    pinMode(J3_MIN_LIMIT_PIN, INPUT_PULLUP);

    //setup stepper motors
    J1Stepper.setMaxSpeed(NEMA17_MAX_SPEED);
    J1Stepper.setAcceleration(NEMA17_MAX_ACCEL);
    J1Stepper.setSpeed(0);
    J2Stepper.setMaxSpeed(NEMA17_MAX_SPEED);
    J2Stepper.setAcceleration(NEMA17_MAX_ACCEL);
    J2Stepper.setSpeed(0);
    J3Stepper.setAcceleration(NEMA17_MAX_ACCEL);
    J3Stepper.setMaxSpeed(NEMA17_MAX_SPEED);
    J3Stepper.setSpeed(0);

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
    int J1_position = getManualInput("Enter J1 angle (+/- 90deg): ", -90, 90);
    int J2_position = getManualInput("Enter z-axis height (192mm, 387mm): ", 192, 387);
    int J3_position = getManualInput("Enter J3 angle (+/- 148deg): ", -148, 148);

    //convert user input into step positions for motors
    int b_step = calculateStep(J1_position, -90, 90, J1_min_step, J1_max_step);
    int J2_step = calculateStep(J2_position, 192, 387, J2_min_step, J2_max_step);
    int e_step = calculateStep(J3_position, -148, 148, J3_min_step, J3_max_step);

    //run motors to step positions
    Serial.print("Moving SCARA to position... ");
    moveAxis(J1Stepper, b_step);
    moveAxis(J2Stepper, J2_step);
    moveAxis(J3Stepper, e_step);
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
    
    // J1Stepper.setSpeed(-1 * NEMA17_MAX_SPEED / 10);
    // while( digitalRead(J3_MIN_LIMIT_PIN)==LOW ){
    //     J1Stepper.runSpeed();
    // }
    // J1_min_step = J1Stepper.currentPosition();

    // J1Stepper.setSpeed(1 * NEMA17_MAX_SPEED / 10);
    // while( digitalRead(J3_MAX_LIMIT_PIN)==LOW ){
    //     J1Stepper.runSpeed();
    // }
    // J1_max_step = J1Stepper.currentPosition();

    //top
    J2Stepper.setSpeed(NEMA17_MAX_SPEED / 3);
    while( digitalRead(J2_MAX_LIMIT_PIN)==LOW ){
        J2Stepper.runSpeed();
    }
    J2_max_step = J2Stepper.currentPosition();

    //bottom
    J2Stepper.setSpeed(-1 * NEMA17_MAX_SPEED / 3);
    while( digitalRead(J2_MIN_LIMIT_PIN)==LOW ){
        J2Stepper.runSpeed();
    }
    J2_min_step = J2Stepper.currentPosition();
        
    //left
    J3Stepper.setSpeed(-1 * NEMA17_MAX_SPEED / 15);
    while( digitalRead(J3_MIN_LIMIT_PIN)==LOW ){
        J3Stepper.runSpeed();
    }
    J3_min_step = J3Stepper.currentPosition();

    //right
    J3Stepper.setSpeed(1 * NEMA17_MAX_SPEED / 15);
    while( digitalRead(J3_MAX_LIMIT_PIN)==LOW ){
        J3Stepper.runSpeed();
    }
    J3_max_step = J3Stepper.currentPosition();

    Serial.println("Done.");

} //end of calibrateLimits()

void zeroHome(){
    //move to center positions
    J2Stepper.moveTo( floor(average(J2_max_step, J2_min_step)) );
    J2Stepper.setSpeed(NEMA17_MAX_SPEED/3);
    J2Stepper.run();
    J3Stepper.moveTo( floor(average(J3_max_step, J3_min_step)) );
    J3Stepper.setSpeed(NEMA17_MAX_SPEED/15);
    J3Stepper.run();
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