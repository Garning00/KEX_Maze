#include <PID_v1.h>
#include "AccelStepper.h"
#include <Filters.h>

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
double KpX = 0.1, KiX = 0, KdX = 0.3;
double KpY = 0.05, KiY = 0, KdY = 0.2;
double conversion = 1.8;

int Ts = 15;

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

float filterFrequencyX = 4;
float filterFrequencyY = 2.3;

FilterOnePole lowpassFilterX(LOWPASS, filterFrequencyX);
FilterOnePole lowpassFilterY(LOWPASS, filterFrequencyY);

void setupMicrostepping() {
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(MS3_1, OUTPUT);

  digitalWrite(MS1_1, HIGH);
  digitalWrite(MS2_1, HIGH);
  digitalWrite(MS3_1, HIGH);

  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);
  pinMode(MS3_2, OUTPUT);

  digitalWrite(MS1_2, HIGH);
  digitalWrite(MS2_2, HIGH);
  digitalWrite(MS3_2, HIGH);
}


void resetPosition() {
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
}

void setup() {
  setupMicrostepping();

  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW);

  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW);

  Serial.setTimeout(2);
  Serial.begin(19200);

  // Initialize position
  resetPosition();

  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(3200);
  stepper2.setMaxSpeed(3200);
  //stepper1.setSpeed(200);
  stepper1.setAcceleration(1000);
  stepper2.setAcceleration(1000);

  // Initialize variables
  //InputX = X_desired;
  //InputY = Y_desired;
  SetpointX = X_desired;  // Center of plane
  SetpointY = Y_desired;  //


  myPIDY.SetMode(AUTOMATIC);
  myPIDX.SetMode(AUTOMATIC);
  myPIDX.SetOutputLimits(-250, 250);  //(-310, 310) // (-6, +7) TODO: Convert to steps?
  myPIDY.SetOutputLimits(-400, 400);  //(-500, 500) // (-10, +8)
  myPIDX.SetSampleTime(Ts);
  myPIDY.SetSampleTime(Ts);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int sep = input.indexOf(' ');
    if (sep > 0) {
      String x_str = input.substring(0, sep);
      String y_str = input.substring(sep + 1);

      X_current = x_str.toFloat();
      Y_current = y_str.toFloat();
    }
  }

  //InputX = lowpassFilterX.input(X_current);
  //InputY = lowpassFilterY.input(Y_current);
  InputX = X_current;
  InputY = Y_current;

  myPIDX.Compute();
  myPIDY.Compute();

  //int targetX = round(OutputX / conversion);
  //int targetY = round(OutputY / conversion);

  stepper1.moveTo(OutputX);
  stepper2.moveTo(OutputY);

  stepper1.run();
  stepper2.run();
}