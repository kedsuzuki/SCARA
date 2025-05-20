/*
@file: scara_alpha_v2.ino
@updated: 2025/02/10
@author: Kensei Suzuki & Diego Gomez
@brief: Alpha version 2.0 meant for development. Top level main code for controlling the SCARA robot. 
        V2.0 includes a new naming convention as well as modularized and re-usable functions.
            J1 --> Base joint
            J2 --> Z-axis joint
            J3 --> Arm joint
*/
// D.G First Comment

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  LIBRARIES | GLOBALS | OBJECTS | FUNCTION PROTOTYPES
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define NEMA17_MAX_SPEED 500.0    //steps per second
#define NEMA17_MAX_ACCEL 50.0     //steps per second^2
#define NEMA17_STEPS_PER_REV 200  //steps per one output revolution
#define MICROSTEP_FACTOR 8        //microstepping - increases steps/rev by factor of {2, 4, 8, 16}

// pins
const int J1_DIR_PIN = 12;  //Revolute joint base NEMA stepper motors control pins
const int J1_STEP_PIN = 13;
const int J2_DIR_PIN = 8;  //Prismatic joint z-axis NEMA stepper motors control pins
const int J2_STEP_PIN = 9;
const int J3_DIR_PIN = 10;  //Revolute joint arm NEMA stepper motors control pins
const int J3_STEP_PIN = 11;
const int J2_MAX_LIMIT_PIN = 4;  //limit switch
const int J2_MIN_LIMIT_PIN = 3;
const int J3_MIN_LIMIT_PIN = 5;
const int J3_MAX_LIMIT_PIN = 6;
const int J1_LIMIT_PIN = 2;
const int GRIPPER_PIN = 7;  //gripper servo motor PWM pin

// link lengths (in mm)

const float L1 = 170;
const float L2 = 98.521;
const float range = 268.521;
float theta1, theta2, Xd, Yd;

// globals
int J1_max_step = 0, J1_min_step = 0, J2_max_step = 0, J2_min_step = 0, J3_min_step = 0, J3_max_step = 0;
int user_inputs[3] = { 0, 0, 0 };

//class objects
AccelStepper J1_Stepper(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper J2_Stepper(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper J3_Stepper(AccelStepper::DRIVER, J3_STEP_PIN, J3_DIR_PIN);
Servo GripperServo;

//func protos

void inv_kin(int, int);
void calibrateLimits();
void zeroHome();
void moveAxis(AccelStepper, int, int);
int getManualInput(const char *, int, int);
int calculateStep(int, int, int, int, int);
bool nosol = false;

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *          SETUP - COMMS, ACTUATORS, SENSORS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void setup() {
  //init comms
  Serial.begin(9600);
  while (!Serial) {}

  //init limit switches
  pinMode(J1_LIMIT_PIN, INPUT_PULLUP);
  pinMode(J2_MAX_LIMIT_PIN, INPUT_PULLUP);
  pinMode(J2_MIN_LIMIT_PIN, INPUT_PULLUP);
  pinMode(J3_MAX_LIMIT_PIN, INPUT_PULLUP);
  pinMode(J3_MIN_LIMIT_PIN, INPUT_PULLUP);

  //setup stepper motors
  J1_Stepper.setMaxSpeed(NEMA17_MAX_SPEED);
  J1_Stepper.setAcceleration(NEMA17_MAX_ACCEL);
  J1_Stepper.setSpeed(0);
  J2_Stepper.setMaxSpeed(NEMA17_MAX_SPEED);
  J2_Stepper.setAcceleration(NEMA17_MAX_ACCEL);
  J2_Stepper.setSpeed(0);
  J3_Stepper.setAcceleration(NEMA17_MAX_ACCEL);
  J3_Stepper.setMaxSpeed(NEMA17_MAX_SPEED);
  J3_Stepper.setSpeed(0);

  delay(250);
  Serial.println("SCARA is initialized.");

  calibrateLimits();
  zeroHome();
}  //end of setup()

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                    LOOP FUNCTION
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
void loop() {
  nosol = false; // resetting flag
  // Serial.println(nosol);                                                                                          // Set boolean flag for theta2 check
  int X_pos = getManualInput("Enter desired position of Robot in the x-direction (+/- 222mm): ", -range, range);  // Calls user to enter position of Robot in x-direction
  delay(1000);
  int Y_pos = getManualInput("Enter desired position of Robot in the y-direction (+/- 222mm): ", -range, range);  // Calls user to enter position of Robot in y-direction
  inv_kin(X_pos, Y_pos);                                                                                          // calls function to calculate angles required to reach desired position
  // Serial.println(nosol);

  if (nosol == true) { // if flag goes off, then program will output there is no solution available
    Serial.println("Sorry! No Solution Exists to Get to Desired Position");
    loop();
  }

  else {
    int J2_position = getManualInput("Enter z-axis height (192mm, 387mm): ", 192, 387);  // asks user to enter desired height of robot in z-dir3ection
    // Serial.println(theta1 * (180 / PI));
    // Serial.println(theta2 * (180 / PI));
    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    int J1_step = calculateStep(theta1, -360, 360, J1_min_step, J1_max_step);  // calculates steps needed to get to desired angles
    int J2_step = calculateStep(J2_position, 192, 387, J2_min_step, J2_max_step);
    int J3_step = calculateStep(theta2, -125, 125, J3_min_step, J3_max_step);
    Serial.print("Moving SCARA to position... ");
    moveAxis(J1_Stepper, J1_step, 1);
    moveAxis(J2_Stepper, J2_step, 1);
    moveAxis(J3_Stepper, J3_step, 1);
    Serial.println("Done");
    Serial.println("You should now be at: ");
    Serial.print(Xd);
    Serial.print(", ");
    Serial.print(Yd);
    Serial.print(", ");
    Serial.print(J2_position);
    Serial.println();
  }
}

/*
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *                USER-DEFINED FUNCTIONS
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

void calibrateLimits() {

  // Determines step count for joint limits using hinge limit switches.

  Serial.print("Calibrating joint limits ... ");

  J1_Stepper.setSpeed(NEMA17_MAX_SPEED);
  while (digitalRead(J1_LIMIT_PIN) == LOW) {  //J1 max - base (revolute)
    J1_Stepper.runSpeed();
  }
  J1_max_step = J1_Stepper.currentPosition();

  J1_Stepper.setSpeed(-NEMA17_MAX_SPEED);
  while (digitalRead(J1_LIMIT_PIN) == LOW) {  //J1 min - base (revolute)
    J1_Stepper.runSpeed();
  }
  J1_min_step = J1_Stepper.currentPosition();

  J2_Stepper.setSpeed(NEMA17_MAX_SPEED);
  while (digitalRead(J2_MAX_LIMIT_PIN) == LOW) {  //J2 max - z-axis (prismatic)
    J2_Stepper.runSpeed();
  }
  J2_max_step = J2_Stepper.currentPosition();

  J2_Stepper.setSpeed(-NEMA17_MAX_SPEED);
  while (digitalRead(J2_MIN_LIMIT_PIN) == LOW) {  //J2 min - z-axis (prismatic)
    J2_Stepper.runSpeed();
  }
  J2_min_step = J2_Stepper.currentPosition();

  J3_Stepper.setSpeed(NEMA17_MAX_SPEED);
  while (digitalRead(J3_MAX_LIMIT_PIN) == LOW) {  //J3 max - arm (revolute)
    J3_Stepper.runSpeed();
  }
  J3_max_step = J3_Stepper.currentPosition();

  J3_Stepper.setSpeed(-NEMA17_MAX_SPEED);
  while (digitalRead(J3_MIN_LIMIT_PIN) == LOW) {  //J3 min - arm (revolute)
    J3_Stepper.runSpeed();
  }
  J3_min_step = J3_Stepper.currentPosition();

  Serial.println("Done.");
}  //end of calibrateLimits()

void zeroHome() {

  // Moves the SCARA to a home or zero position. HOME is defined as the all joints at a middle position away from any self-collisions.


  Serial.print("Moving to HOME position.....");
  int J1_home = floor((abs(J1_max_step) + abs(J1_min_step)) / 2);
  int J2_home = floor((abs(J2_max_step) + abs(J2_min_step)) / 2);
  int J3_home = floor((abs(J3_max_step) + abs(J3_min_step)) / 2);

  J1_Stepper.setSpeed(NEMA17_MAX_SPEED);
  J2_Stepper.setSpeed(NEMA17_MAX_SPEED);
  J3_Stepper.setSpeed(NEMA17_MAX_SPEED);

  J1_Stepper.moveTo(J1_home);
  J2_Stepper.moveTo(J2_home);
  J3_Stepper.moveTo(J3_home);

  while (J1_Stepper.distanceToGo() != 0 || J2_Stepper.distanceToGo() != 0 || J3_Stepper.distanceToGo() != 0) {
    J1_Stepper.run();
    J2_Stepper.run();
    J3_Stepper.run();
  }

  Serial.println("Done");
}  //end of zeroHome()

void moveAxis(AccelStepper joint, int stp, int speed_reduction) {
  int d = 1;
  joint.currentPosition() > stp ? d = 1 : d = -1;
  joint.setSpeed(d * NEMA17_MAX_SPEED / speed_reduction);
  joint.moveTo(stp);
  while (joint.distanceToGo() != 0) {
    Serial.println("moving?");
    joint.run();
  }
}  //end of moveAxis()

int getManualInput(const char *prompt, int minVal, int maxVal) {
  int input;

  do {
    Serial.println(prompt);
    while (!Serial.available()) {}
    input = Serial.parseInt();
    if (input < minVal || input > maxVal) {
      Serial.println("Invalid input. Please try again.");
    }
  } while (input < minVal || input > maxVal);

  return input;
}  //end of getManualInput();

int calculateStep(int input, int minInput, int maxInput, int minStep, int maxStep) {
  return map(input, minInput, maxInput, minStep, maxStep);
}  //end of calculateStep()

void inv_kin(int x, int y) {
  theta2 = acos((pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));  // calculates theta2 using trig functions

  // Depending on quadrant where desired robot location is located, inverse kineamtics function adjusts theta1 & theta2
  // First Quadrant
  if (x > 0 && y > 0) {
    Serial.println("Running 1st Option");
    theta1 = PI / 2 + (atan2(y, (x)) - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2))));
    theta2 = -theta2;
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));
  }

  // 2nd Quadrant (if(x) <= 0 && y >= 0)
  // float theta1 = (atan2(y,(x))*180/PI) - (90 - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2))) * 180/PI);
  if (x <= 0 && y >= 0) {
    Serial.println("Running 2nd Option");
    float theta1 = atan2(y, (x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2))));
  }

  // 3rd Quadrant (if(x) <= 0 && y <= 0)
  // float theta1 = (atan2(y,(x)) * 180/PI) - (90 - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2))) * 180/PI) + 360 ;
  if (x <= 0 && y < 0) {
    Serial.println("Running 3rd Option");
    theta1 = atan2(y, (x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2)))) + 2 * PI;
  }

  // 4th Quadrant
  // float theta1 = (atan2(y,(x))*180/PI) - (90 - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2))) * 180/PI) + 360;
  if (x > 0 && y <= 0) {
    Serial.println("Running 4th Option");
    theta1 = atan2(y, (x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2)))) + 2 * PI;
  }

  if (abs(theta2 * (180 / PI)) > 125) { // flag that checks that theta 2 (angle of L2) is less than the allowable limit set by switch
    nosol = true; // if it is greater than, it will set the flag off
  }
}



/* theta1 = PI / 2 - (atan2(y,(x)) - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2))));
    theta2 = -theta2;
    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));

    Serial.println(Xd);
    Serial.println(Yd);
    if (x) != Xd || y != Yd) {
      Serial.println('A');
      theta2 = -theta2;
      theta1 = PI / 2 + (atan2(y,(x)) - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2))));
      theta2 = -theta2;
      Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
      Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
      Serial.println(theta1 * (180 / PI));
      Serial.println(theta2 * (180 / PI));
      Serial.println(Xd);
      Serial.println(Yd);
    } else {
      Serial.println("You have arrived at your desired position");
    }


    theta2 = acos((pow(x), 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  // Need to include that if theta2 is greater than limits of L2, to return to Manual Input function

  // First Quadrant
  // float theta1 = 90 - (atan2(y,(x)) - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2)))) * 180/PI;
  if (x > 0 && y > 0) {
    Serial.println("Running 1st Option");
    theta1 = PI / 2 + (atan2(y,(x)) - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2))));
    theta2 = -theta2;
    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));
    Serial.println(Xd);
    Serial.println(Yd);
  }

  // 2nd Quadrant (if(x) <= 0 && y >= 0)
  // float theta1 = (atan2(y,(x))*180/PI) - (90 - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2))) * 180/PI);
  if (x) <= 0 && y >= 0) {
    Serial.println("Running 2nd Option");
    float theta1 = atan2(y,(x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2))));

    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));
    Serial.println(Xd);
    Serial.println(Yd);
  }

  // 3rd Quadrant (if(x) <= 0 && y <= 0)
  // float theta1 = (atan2(y,(x)) * 180/PI) - (90 - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2))) * 180/PI) + 360 ;
  if (x) <= 0 && y < 0) {
    Serial.println("Running 3rd Option");
    theta1 = atan2(y,(x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2)))) + 2 * PI;
    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));
    Serial.println(Xd);
    Serial.println(Yd);
  }

  // 4th Quadrant
  // float theta1 = (atan2(y,(x))*180/PI) - (90 - atan2((L1 + L2*(cos(theta2))), L2*(sin(theta2))) * 180/PI) + 360;
  if (x) > 0 && y <= 0) {
    Serial.println("Running 4th Option");
    theta1 = atan2(y,(x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2)))) + 2 * PI;
    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));
    Serial.println(Xd);
    Serial.println(Yd);
  }
    */