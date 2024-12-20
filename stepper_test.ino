/*
@file: stepper_test.ino
@updated: 2024/12/18
@author: Kensei Suzuki
@brief: Testing utility of AccelStepper library.
    1. How are step counts recorded - continous or reset after full revolution?
    2. Which is better: position control or speed control?
@resources:
    https://www.pjrc.com/teensy/td_libs_AccelStepper.html
    https://www.airspayce.com/mikem/arduino/AccelStepper/ 
    https://github.com/waspinator/AccelStepper 
*/


/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  LIBRARIES | GLOBALS | OBJECTS | FUNCTION PROTOTYPES
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

#define POSITION_CTRL           1
#define SPEED_CTRL              0

#include <AccelStepper.h>

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution
#define SPIN_DIRECTION          1         //1 or -1 for CW or CCW

const int DIR_PIN               = 7;        //nema stepper motors control pins
const int STEP_PIN              = 8;

AccelStepper MyStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *          SETUP - COMMS, ACTUATORS, SENSORS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void setup() {
    Serial.begin(9600);
    while(!Serial){}

    MyStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    MyStepper.setAcceleration(NEMA17_MAX_ACCEL);
    MyStepper.setSpeed(0);
    
    #if POSITION_CTRL   
    MyStepper.setSpeed(NEMA17_MAX_SPEED/5);
    #endif

    delay(100);
    Serial.println("Start Test");
}

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop() {
    Serial.println("")
    
    #if POSITION_CTRL
    for(int i=0; i<5; i++){
        Serial.print("Target Position: "); Serial.println(90*i);
        MyStepper.moveTo(90*i);
        MyStepper.runToPosition();
        delay(500);
    }
    #endif

    #if SPEED_CTRL
    MyStepper.setSpeed(SPIN_DIRECTION * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    MyStepper.runSpeed();
    #endif

}