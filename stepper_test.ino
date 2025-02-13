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

#define POSITION_CTRL           1
#define SPEED_CTRL              0

#include <AccelStepper.h>

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution

const int DIR_PIN               = 7;        //nema stepper motors control pins
const int STEP_PIN              = 8;

int spin_dir = 1;

AccelStepper MyStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

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
    
    #if POSITION_CTRL
    Serial.println("Start Position Control Test");
    #endif

    #if SPEED_CTRL
    Serial.println("Start Speed Control Test");
    #endif
}

void loop() {
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
    speed_dir *= -1; //reverse direction
    #endif

}