/*
@file: scara_alpha_v1_0.ino
@updated: 2024/11/30
@author: Kensei Suzuki & Diego Gomez
@brief: Alpha version meant for development. Top level main code for controlling the SCARA robot. 
*/

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  LIBRARIES | GLOBALS | OBJECTS | FUNCTION PROTOTYPES
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define JOG_BASE 0
#define JOG_ZAXIS 0
#define JOG_ELBOW 0 
#define ACTUATE_GRIPPER 1

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution
#define SPEED_REDUCTION         10         //reduce nema speed by this factor

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
int top_lim_step, bottom_lim_step, left_lim_step, right_lim_step;

//class objects
AccelStepper BaseStepper(AccelStepper::DRIVER, BASE_STEP_PIN, BASE_DIR_PIN);
AccelStepper ZStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper ElbowStepper(AccelStepper::DRIVER, ELBOW_STEP_PIN, ELBOW_DIR_PIN);
Servo GripperServo;

//func protos


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
    ElbowStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    ElbowStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ElbowStepper.setSpeed(0);

    delay(250);
} //end of setup()

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop(){
    #if JOG_BASE
    if( digitalRead(BOTTOM_LIMIT_PIN)==HIGH ){
        BaseStepper.setSpeed(1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
        //cw looking down (-)
    }    
    else if( digitalRead(TOP_LIMIT_PIN)==HIGH ){
        BaseStepper.setSpeed(-1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
        //ccw looking down (+)
    }
    else{
        BaseStepper.setSpeed(0);
    }
    BaseStepper.runSpeed();
    #endif

    #if JOG_ZAXIS
    if( digitalRead(BOTTOM_LIMIT_PIN)==HIGH ){
        ZStepper.setSpeed(-1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }    
    else if( digitalRead(TOP_LIMIT_PIN)==HIGH ){
        ZStepper.setSpeed(1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }
    else{
        ZStepper.setSpeed(0);
    }
    ZStepper.runSpeed();
    #endif

    #if JOG_ELBOW
    //speed reduction 10
    if( digitalRead(LEFT_LIMIT_PIN)==HIGH ){
        ElbowStepper.setSpeed(-1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }    
    else if( digitalRead(RIGHT_LIMIT_PIN)==HIGH ){
        ElbowStepper.setSpeed(1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }
    else{
        ElbowStepper.setSpeed(0);
    }
    ElbowStepper.runSpeed();
    #endif

    #if ACTUATE_GRIPPER
    int pos = GripperServo.read();
    Serial.println(pos);
    if( digitalRead(LEFT_LIMIT_PIN)==HIGH ){
        pos -= 1;
        GripperServo.write(pos);
        delay(100);
    }
    else if( digitalRead(RIGHT_LIMIT_PIN)==HIGH){
        pos += 1;
        GripperServo.write(pos);
        delay(100);
        }
    #endif


} //end of loop()

