#include <PID_v1.h>
#include "AccelStepper.h"

#define dirPin1 2
#define stepPin1 3

#define dirPin2 8
#define stepPin2 9

#define motorInterfaceType 1



#define MS1_1 4
#define MS2_1 5
#define MS3_1 6
#define ENABLE_PIN_1 7

#define MS1_2 10
#define MS2_2 11
#define MS3_2 12
#define ENABLE_PIN_2 13



double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

// Constants V1
//double KpX = 0.07, KiX = 0.1, KdX = 0.05;
//double KpY = 0.13, KiY = 0.03, KdY = 0.045;
//double KpY = 0.07, KiY = 0.03, KdY = 0.065;

// Constants V2
//double KpX = 0.008, KiX = 0.01, KdX = 0.02;
//double KpY = 0.007, KiY = 0.01, KdY = 0.01;
//double KpY = 0.013, KiY = 0.02, KdY = 0.03;
//double KpX = 0.0008, KiX = 0, KdX = 0;
double KpY = 0.0001, KiY = 0, KdY = 0;

// Constants V3 (litet P --> ingen overshoot, I för e0, D för dämpa oscillation)
double KpX = 0.0115, KiX = 0.005, KdX = 0;

double conversion = 1.8;
int stepMode = 2;

int Ts = 10;

int InputXfiltered, InputYfiltered;

// Center Point
int X_desired = 539, Y_desired = 405;

// Initialize variables
int X_current = X_desired, Y_current = Y_desired;


PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

void setupMicrostepping() {
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(MS3_1, OUTPUT);

  digitalWrite(MS1_1, HIGH);
  digitalWrite(MS2_1, LOW);
  digitalWrite(MS3_1, LOW);

  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);
  pinMode(MS3_2, OUTPUT);

  digitalWrite(MS1_2, HIGH);
  digitalWrite(MS2_2, LOW);
  digitalWrite(MS3_2, LOW);
}




void resetPosition(){
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
}

void setup() {
  setupMicrostepping();

  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW);

  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW);

  Serial.begin(19200);
  Serial.flush();

  // Initialize position
  resetPosition();

  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(200*stepMode);
  stepper2.setMaxSpeed(200*stepMode);
  //stepper1.setSpeed(200);
  stepper1.setAcceleration(250*stepMode); // 1000
  stepper2.setAcceleration(250*stepMode);

 // Initialize variables
  //InputX = X_desired;
  //InputY = Y_desired;
  SetpointX = X_desired;  // Center of plane
  SetpointY = Y_desired;  //


  myPIDY.SetMode(AUTOMATIC);
  myPIDX.SetMode(AUTOMATIC);
  myPIDX.SetOutputLimits(-35*stepMode,35*stepMode); // (-6, +7) TODO: Convert to steps?
  myPIDY.SetOutputLimits(-15*stepMode,15*stepMode);// (-10, +8)
  myPIDX.SetSampleTime(Ts);
  myPIDY.SetSampleTime(Ts);
}

void loop() {
  if (Serial.available()) {
    X_current = Serial.parseFloat();
    Y_current = Serial.parseFloat();
    X_desired = Serial.parseFloat();
    Y_desired = Serial.parseFloat();

    if (X_desired != "-69" && Y_desired != "-69"){
        SetpointX = X_desired;
        SetpointY = Y_desired;
      }

    InputX = X_current;
    InputY = Y_current;

    while (Serial.available() > 0){
        Serial.read();
      }
    }

  myPIDX.Compute();
  myPIDY.Compute();

  //int targetX = round(OutputX / conversion);
  //int targetY = round(OutputY / conversion);

  stepper1.moveTo(OutputX);
  stepper2.moveTo(OutputY);

  stepper1.run();
  stepper2.run();


}
