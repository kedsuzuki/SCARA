/*
@file: scara_alpha_v1_0.ino
@updated: 2024/11/16
@author: Kensei Suzuki & Diego Gomez
@brief: Alpha version meant for development. Top level main code for controlling the SCARA robot.
*/


/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  LIBRARIES | GLOBALS | OBJECTS | FUNCTION PROTOTYPES
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
#include <AccelStepper.h>

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution
#define SPEED_REDUCTION         7         //reduce nema speed by this factor
#define RAD_TO_DEG              180.0/PI  //radian to degree conversion (180/pi=57.296)

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

//class objects
AccelStepper BaseStepper(AccelStepper::DRIVER, BASE_STEP_PIN, BASER_DIR_PIN);
AccelStepper ZStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper ElbowStepper(AccelStepper::DRIVER, ELBOW_STEP_PIN, ELBOW_DIR_PIN);

//func protos
void calibrateLimits();
void zeroHome();
void requestOpMode();

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *          SETUP - COMMS, ACTUATORS, SENSORS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void setup(){
    //init comms
    Serial.begin(115200);
    while(!Serial){}

    //init limit switches
    pinMode(TOP_LIMIT_PIN, INPUT); //make input pulldown to set default to LOW?
    pinMode(BOTTOM_POT_PIN, INPUT);
    pinMode(RIGHT_POT_PIN, INPUT);
    pinMode(LEFT_POT_PIN, INPUT);

    //setup stepper motors
    BaseStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    BaseStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ZStepper.setSpeed(200);
    ZStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    ElbowStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ElbowStepper.setSpeed(200);

    delay(250);
} //end of setup()

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop(){
    
} //end of loop()

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                USER-DEFINED FUNCTIONS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

void calibrateLimits(){
  //top
  
  //bottom

  //left

  //right
  
}

void zeroHome(){
}
  

}

void moveAxis(&Stepper motor, float pos){
}