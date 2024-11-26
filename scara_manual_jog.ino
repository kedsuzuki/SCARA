/*
@file: scara_alpha_v1_0.ino
@updated: 2024/11/25
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

#define JOG_BASE 1
#define JOG_ZAXIS 0
#define JOG_ELBOW 0 
#define ACTUATE_GRIPPER 0

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution
#define SPEED_REDUCTION         5         //reduce nema speed by this factor

// pins
const int BASE_DIR_PIN        = 2;        //nema stepper motors control pins
const int BASE_STEP_PIN       = 3;
const int Z_DIR_PIN           = 4;
const int Z_STEP_PIN          = 5;
const int ELBOW_DIR_PIN       = 4;
const int ELBOW_STEP_PIN      = 5;
const int TOP_LIMIT_PIN       = 6;        //limit switch 
const int BOTTOM_LIMIT_PIN    = 7;
const int LEFT_LIMIT_PIN      = 8;
const int RIGHT_LIMIT_PIN     = 9;
const int GRIPPER_PIN         = 11;       //gripper servo motor PWM pin

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
    pinMode(TOP_LIMIT_PIN, INPUT);
    pinMode(BOTTOM_LIMIT_PIN, INPUT);
    pinMode(RIGHT_LIMIT_PIN, INPUT);
    pinMode(LEFT_LIMIT_PIN, INPUT);

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
    if( digitalRead(BOTTOM_LIMIT_PIN)==LOW ){
        BaseStepper.setSpeed(-1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }    
    else if( digitalRead(TOP_LIMIT_PIN)==LOW ){
        BaseStepper.setSpeed(1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }
    else{
        BaseStepper.setSpeed(0);
    }
    BaseStepper.runSpeed();
    #endif

    #if JOG_ZAxis
    if( digitalRead(BOTTOM_LIMIT_PIN)==LOW ){
        ZStepper.setSpeed(-1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }    
    else if( digitalRead(TOP_LIMIT_PIN)==LOW ){
        ZStepper.setSpeed(1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }
    else{
        ZStepper.setSpeed(0);
    }
    ZStepper.runSpeed();
    #endif

    #if JOG_ELBOW
    if( digitalRead(LEFT_LIMIT_PIN)==LOW ){
        ElbowStepper.setSpeed(-1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }    
    else if( digitalRead(RIGHT_LIMIT_PIN)==LOW ){
        ElbowStepper.setSpeed(1 * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    }
    else{
        ElbowStepper.setSpeed(0);
    }
    ElbowStepper.runSpeed();
    #endif

    #if ACTUATE_GRIPPER
    int pos = GripperServo.read();
    if( digitalRead(LEFT_LIMIT_PIN)==LOW ){
        pos--;
        GripperServo.write(pos);
        delay(10);
    }
    else if( digitalRead(RIGHT_LIMIT_PIN)==LOW){
        pos++;
        GripperServo.write(pos);
        delay(10);
        }
    #endif


} //end of loop()

