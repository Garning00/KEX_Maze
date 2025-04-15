#include <PID_v1.h>
#include "AccelStepper.h"

#define dirPin1 2
#define stepPin1 3
#define dirPin2 8
#define stepPin2 9
#define motorInterfaceType 1

double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;
double KpX = 1, KiX = 0, KdX = 0; 
double KpY = 1, KiY = 0, KdY = 0; 
double conversion = 1.8;

int StepsX = 0, StepsY = 0;

int Ts = 15;


int X_current = 0, Y_current = 0;
int X_desired = 114, Y_desired = 136; 

PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, REVERSE);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, REVERSE);

// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);


void resetPosition(){
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
}

void setup() {
  Serial.begin(19200); 
	// Serial.setTimeout(1); 

  Serial.flush();

  // Initialize position
  resetPosition();

  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(200);
  stepper2.setMaxSpeed(200);
  //stepper1.setSpeed(200);
  stepper1.setAcceleration(500);
  stepper2.setAcceleration(500);

  InputX = 0;
  InputY = 0;
  SetpointX = 0; // Center of plane
  SetpointY = 0; // 
  

  myPIDY.SetMode(AUTOMATIC);
  myPIDX.SetMode(AUTOMATIC);
  myPIDX.SetOutputLimits(-14,18); // (-6, +7) TODO: Convert to steps?
  myPIDY.SetOutputLimits(-32,35);// (-10, +8) 
  myPIDX.SetSampleTime(Ts);
  myPIDY.SetSampleTime(Ts);
}

void loop() {

  if (Serial.available()){
    X_current = Serial.parseFloat();
    Y_current = Serial.parseFloat();

    while (Serial.available() > 0){
      Serial.read();
    }

  }

  InputX = X_current - X_desired; 
  InputY = Y_current - Y_desired; 

  // if (InputX < tol_x && InputY < tol_y)

  myPIDX.Compute();
  myPIDY.Compute();
  //adjust motor angle for both X and Y

  //StepsX = round(OutputX/conversion);
  //StepsY = round(OutputY/conversion);

  if (stepper1.currentPosition() != StepsX || stepper2.currentPosition() != StepsY){
    stepper1.moveTo(OutputX);
    stepper2.moveTo(OutputY);
    stepper1.run();
    stepper2.run();
    //stepper1.runToPosition();
  }  
}
