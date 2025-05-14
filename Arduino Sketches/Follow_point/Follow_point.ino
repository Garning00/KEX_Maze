#include <PID_v1.h>
#include "AccelStepper.h"
//#include <Filters.h>

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
//double KpY = 0.0001, KiY = 0, KdY = 0;

// Constants V3 (litet P --> ingen overshoot, I för e0, D för dämpa oscillation)
//double KpX = 0.0115, KiX = 0.005, KdX = 0;

// Constants (slow stable solve)
//double KpX = 0.085, KiX = 0.007, KdX = 0.052;
//double KpY = 0.09, KiY = 0.08, KdY = 0.07;

//Ver 4 (KEXPO)
double KpX = 0.15, KiX = 0.0075, KdX = 0.052;
double KpY = 0.15, KiY = 0.085, KdY = 0.07;

//double KpX = 0.05, KiX = 0.0005, KdX = 0.06;

// Constants fast solve (crash into wall, needs good waypoint placement)
//double KpX = 0.5, KiX = 0.1, KdX = 0;
//double KpY = 0, KiY = 0, KdY = 0;


double conversion = 1.8;
int stepMode = 2;

int Ts = 33.333; // SerialQuery is sent 30times/s also camera fps is 30 (1000/30=33.3)
                 // Although message only received on arduino every 20-40 ms, mostly 33 ms, risk of missing a message? (before w 19200 baud: 50-70 ms)

int InputXfiltered, InputYfiltered;

// Initial
int X_desired = 0, Y_desired = 0;

// Initialize variables
int X_current = X_desired, Y_current = Y_desired;
bool firstInputRecieved = 0;

unsigned long timeMessage, lastMessage, timeSinceLastMessage;
unsigned long timeTimeout = 2000;

PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT); // direction either DIRECT or REVERSE
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

// Initializing filters
//float filterFrequencyX = 0.5;
//float filterFrequencyY = 2.5;

// Initializing a one pole (RC) lowpass filter
//FilterOnePole lowpassFilterX(LOWPASS, filterFrequencyX);
//FilterOnePole lowpassFilterY(LOWPASS, filterFrequencyY);

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

  Serial.begin(38400);//19200);
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

  Serial.write("Starting!\n");
}

void loop() {
  if (Serial.available()) {
    timeMessage = millis();
    timeSinceLastMessage = timeMessage - lastMessage;
    Serial.write("Time since last message: ");
    Serial.println(timeSinceLastMessage);

    X_current = Serial.parseInt(); // ParseFloat if sending in mm and ParseInt if pixel
    Y_current = Serial.parseInt();
    X_desired = Serial.parseInt();
    Y_desired = Serial.parseInt();

    SetpointX = X_desired;
    SetpointY = Y_desired;
    InputX = X_current;
    InputY = Y_current;

    // Filtering touchpanel signal
    //InputX = lowpassFilterX.input(InputX);
    //InputY = lowpassFilterY.input(InputY);
    Serial.write("Recieved values: ");
    Serial.print(X_current);
    Serial.write(",");
    Serial.print(Y_current);
    Serial.write(" ");
    Serial.print(X_desired);
    Serial.write(",");
    Serial.println(Y_desired);

    if (!firstInputRecieved){
    firstInputRecieved = 1;
    resetPosition();
    }

    while (Serial.available() > 0){
        //Serial.read();
        char c = Serial.read();
        if (c == '\n') break;  // Discard remaining junk in line
      }

    // LÄGG TILL TIME SINCE LAST MESSAGE
    // --> stänger av sig själv efter ett tag.
    lastMessage = timeMessage;
    }

  // Avoid movements before first desired position is recieved
  if (firstInputRecieved && millis() - lastMessage < timeTimeout){

    myPIDX.Compute();
    myPIDY.Compute();

    //int targetX = round(OutputX / conversion);
    //int targetY = round(OutputY / conversion);


    if (stepper1.currentPosition() != OutputX || stepper2.currentPosition() != OutputY){
      Serial.write("PID computed output: ");
      Serial.print(OutputX);
      Serial.write(",");
      Serial.println(OutputY);

      // Pid kan ge decimalvärde --> kanske behöver round

      stepper1.moveTo(OutputX);
      stepper2.moveTo(OutputY);
      //Serial.print(OutputX);

      stepper1.run();
      stepper2.run();
    }
  }


}