#include <PID_v1.h>
#include "AccelStepper.h"
#include <Filters.h>

#define dirPin1 2
#define stepPin1 3
#define dirPin2 8
#define stepPin2 9
#define motorInterfaceType 1

double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;
double KpX = 1, KiX = 0.5, KdX = 0.5; 
double KpY = 1, KiY = 0.5, KdY = 0.5; 
double conversion = 1.8;

int Ts = 15;

int InputXfiltered, InputYfiltered;

int X_current = 504, Y_current = 360;


PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

float filterFrequencyX= 4;
float filterFrequencyY= 2.3;

FilterOnePole lowpassFilterX( LOWPASS, filterFrequencyX );
FilterOnePole lowpassFilterY( LOWPASS, filterFrequencyY );


void resetPosition(){
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
}

void setup() {
  Serial.setTimeout(1); 
  Serial.begin(19200); 

  // Initialize position
  resetPosition();

  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(200);
  stepper2.setMaxSpeed(200);
  //stepper1.setSpeed(200);
  stepper1.setAcceleration(500);
  stepper2.setAcceleration(500);

  InputX = 504;
  InputY = 360;
  SetpointX = 504; // Center of plane
  SetpointY = 360; // 
  

  myPIDY.SetMode(AUTOMATIC);
  myPIDX.SetMode(AUTOMATIC);
  myPIDX.SetOutputLimits(-14,18); // (-6, +7) TODO: Convert to steps?
  myPIDY.SetOutputLimits(-32,35);// (-10, +8) 
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
    String x_str = input.substring(0, sep);
    String y_str = input.substring(sep + 1);


    X_current = x_str.toFloat();
    Y_current = y_str.toFloat();

    InputX = lowpassFilterX.input(X_current);
    InputY = lowpassFilterY.input(Y_current);
    
  
    myPIDX.Compute();
    myPIDY.Compute();
  
    int targetX = round(OutputX / conversion);
    int targetY = round(OutputY / conversion);
  
    stepper1.moveTo(targetX);
    stepper2.moveTo(targetY);
  
    stepper1.run();
    stepper2.run();
  }

  
}
