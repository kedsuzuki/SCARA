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

#define NEMA17_MAX_SPEED        500.0     //steps per second
#define NEMA17_MAX_ACCEL        50.0      //steps per second^2
#define NEMA17_STEPS_PER_REV    200       //steps per one output revolution
#define SPEED_REDUCTION         7         //reduce nema speed by this factor
#define RAD_TO_DEG              180.0/PI  //radian to degree conversion (180/pi=57.296)

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
AccelStepper BaseStepper(AccelStepper::DRIVER, BASE_STEP_PIN, BASER_DIR_PIN);
AccelStepper ZStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper ElbowStepper(AccelStepper::DRIVER, ELBOW_STEP_PIN, ELBOW_DIR_PIN);
Servo GripperServo;

//func protos
void calibrateLimits();
void zeroHome();
int getManualInput_Base();
int getManualInput_ZAxis();
int getManualInput_Elbow();
int calculateBaseStep(int);
int calculateZStep(int);
int calculateElbowStep(int);
void moveAxis_Base(int);
void moveAxis_ZAxis(int);
void moveAxis_Elbow(int);

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
    ZStepper.setMaxSpeed(NEMA17_MAX_SPEED);
    ZStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ElbowStepper.setAcceleration(NEMA17_MAX_ACCEL);
    ElbowStepper.setMaxSpeed(NEMA17_MAX_SPEED);

    delay(250);
} //end of setup()

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop(){
    //get user input for each joint 
    int base_position = getManualInput_Base();
    int z_position = getManualInput_ZAxis();
    int elbow_position = getManualInput_Elbow();

    //convert user input into step positions for motors
    int b_step = calculateBaseStep(base_position);
    int z_step = calculateZStep(z_position);
    int e_step = calculateElbowStep(elbow_position);

    //run motors to step positions
    moveAxis_Base(b_step);
    moveAxis_ZAxis(z_step);
    moveAxis_Elbow(e_step);

    delay(100);
} //end of loop()


/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                USER-DEFINED FUNCTIONS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

void calibrateLimits(){
  //top
  while(1){
    digitalRead(TOP_LIMIT_PIN) == HIGH ? break : void()0
    ZStepper.setSpeed(NEMA17_MAX_SPEED / SPEED_REDUCTION);
    ZStepper.runSpeed();
  }
  top_lim_step = ZStepper.currentPosition();

  //bottom
  while(1){
    digitalRead(BOTTOM_LIMIT_PIN) == HIGH ? break : void()0
    ZStepper.setSpeed(NEMA17_MAX_SPEED / 10);
    ZStepper.runSpeed();
  }
  bottom_lim_step = ZStepper.currentPosition();
  
  //left
  while(1){
    digitalRead(LEFT_LIMIT_PIN) == HIGH ? break : void()0
    ElbowStepper.setSpeed(NEMA17_MAX_SPEED / 10);
    ElbowStepper.runSpeed();
  }
  top_lim_step = ElbowStepper.currentPosition();
  
  //right
  while(1){
    digitalRead(RIGHT_LIMIT_PIN) == HIGH ? break : void()0
    ElbowStepper.setSpeed(NEMA17_MAX_SPEED / 10);
    ElbowStepper.runSpeed();
  }
  top_lim_step = ElbowStepper.currentPosition();
}

void zeroHome(){
  ZStepper.moveTo( floor((top_lim_step+bottom_lim_step)/2) );
  ZStepper.run();
  ElbowStepper.moveTo( (floor((right_lim_step+left_lim_step)/2)) );
  ElbowStepper.run();
}

// void moveAxis(&Stepper motor, float pos){
  // todo/ks: generalized funtion to move any of the joints by passing a reference to the stepper object
  // and the position to be moved to
// }

int getManualInput_Base(){
  Serial.println("Enter position to move to: ");

  while( Serial.available() == 0 ){}
  int InputAngle = Serial.parseInt()

  if InputAngle > 360 || InputAngle < 0 {
    Serial.println("Please input a number between 0 and 360");
  }

  else {
    return InputAngle;
  }
}

int getManualInput_ZAxis(){
  Serial.println("Enter position to move to: ");

  while( Serial.available() == 0 ){}
  int InputAngle = Serial.parseInt()

  if InputAngle > 387 || InputAngle < 192 {
    Serial.println("Please input a number between 192 and 387");
  }

  else {
    return InputAngle;
  }
}

int getManualInput_Elbow(){
  Serial.println("Enter position to move to: ");

  while( Serial.available() == 0 ){}
  int InputAngle = Serial.parseInt()

  if abs(InputAngle) > 148 {
    Serial.println("Please input a number between 0 and 148");
  }

  else {
    return InputAngle;
  }
}

int calculateBaseStep(int pos){
  //computes the step required for the base motor to run to achieve user input position angle
  
  return 0;
}

int calculateZStep(int pos){
  //computes the step the Z-axis motor must run to in order to achieve user input position height
  int stp = map(pos, 192, 387, left_lim_step, right_lim_step);
  return stp
}

int calculateElbowStep(int pos){
  //computes the step required for the elbow motor to run to the user input position angle
  int stp = map(pos, -148, 148, bottom_lim_step, top_lim_step);
  return stp
}

moveAxis_Base(int step){
  BaseStepper.moveTo(step);
  BaseStepper.run();
}

moveAxis_ZAxis(int step){
  ZStepper.moveTo(step);
  ZStepper.run();
}

moveAxis_Elbow(int step){
  ElbowStepper.moveTo(step);
  ElbowStepper.run();
}