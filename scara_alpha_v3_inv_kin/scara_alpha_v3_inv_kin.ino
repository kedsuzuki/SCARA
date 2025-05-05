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
int getManualInput(const char *, int, int);
int calculateStep(int, int, int, int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  int X_pos = getManualInput("Enter desired position of Robot in the x-direction (+/- 222mm): ", -range, range);
  delay(1000);
  int Y_pos = getManualInput("Enter desired position of Robot in the y-direction (+/- 222mm): ", -range, range);
  inv_kin(X_pos, Y_pos);

  
}

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

void inv_kin(int x,int y)
{
  theta2 = acos((pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
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
  if (x <= 0 && y >= 0) {
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
  if (x <= 0 && y < 0) {
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
  if (x > 0 && y <= 0) {
    Serial.println("Running 4th Option");
    theta1 = atan2(y,(x)) - (PI / 2 - atan2((L1 + L2 * (cos(theta2))), L2 * (sin(theta2)))) + 2 * PI;
    Xd = round(L1 * (cos(theta1)) + L2 * cos(theta1 + theta2));
    Yd = round(L1 * (sin(theta1)) + L2 * sin(theta1 + theta2));
    Serial.println(theta1 * (180 / PI));
    Serial.println(theta2 * (180 / PI));
    Serial.println(Xd);
    Serial.println(Yd);
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