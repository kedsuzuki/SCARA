/*
@file: scara_unit_test.ino
@updated: 2024/11/30
@author: Kensei Suzuki & Diego Gomez
@brief: Unit tests to check functionality of individual systems of the SCARA robot.
Modify boolean definitions (0 or 1) to change which test to run. 
*/


/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  LIBRARIES | GLOBALS | OBJECTS | FUNCTION PROTOTYPES
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

#define TEST_STEPPER      0       //moves one stepper using pins 7 and 8. Rotates left and right.
#define TEST_SERVO        0
#define TEST_LIMITSWITCH  1

#include <AccelStepper.h>
#include <Servo.h>

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution
#define SPEED_REDUCTION         5         //reduce nema speed by this factor
#define SPIN_DIRECTION          1         //1 or -1 for CW or CCW

const int DIR_PIN           = 7;        //nema stepper motors control pins
const int STEP_PIN          = 8;
const int LIMIT_SWITCH_PIN  = 3;        //limit switch 
const int SERVO_PIN         = 11;       //gripper servo motor PWM pin

AccelStepper MyStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Servo MyServo;

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *          SETUP - COMMS, ACTUATORS, SENSORS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void setup() {
  MyStepper.setMaxSpeed(NEMA17_MAX_SPEED);
  MyStepper.setAcceleration(NEMA17_MAX_ACCEL);
  MyStepper.setSpeed(0);
  MyStepper.runSpeed();
  
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  MyServo.attach(SERVO_PIN);
  MyServo.write(0);

  delay(100);
}

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop() {
  #if TEST_STEPPER
    MyStepper.setSpeed(SPIN_DIRECTION * NEMA17_MAX_SPEED / SPEED_REDUCTION);
    MyStepper.runSpeed();
  #endif

  #if TEST_LIMITSWITCH
    if(digitalRead(LIMIT_SWITCH_PIN) == HIGH ){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Click");
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Waiting...");
    }
  #endif
  
  #if TEST_SERVO
    for (int cw = 0; cw <= 180; cw += 1) { 
      // goes from 0 degrees to 180 degrees
      MyServo.write(cw);         
      delay(15);
      Serial.print("Clockwise: "); Serial.println(cw);
    }                 
    for (int ccw = 180; ccw >= 0; ccw -= 1) { 
      // goes from 180 degrees to 0 degrees
      MyServo.write(ccw);              
      delay(15);
      Serial.print("Counter-Clockwise"); Serial.println(ccw);                    
    }   
  #endif
}