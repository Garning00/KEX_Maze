/* Example sketch to control a stepper motor with 
   DRV8825 stepper motor driver, AccelStepper library 
   and Arduino: continuous rotation. 
   More info: https://www.makerguides.com */

#include "AccelStepper.h"

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin1 2
#define stepPin1 3


#define dirPin2 8
#define stepPin2 9


#define MS1_1 4
#define MS2_1 5
#define MS3_1 6
#define ENABLE_PIN_1 7

#define MS1_2 10
#define MS2_2 11
#define MS3_2 12
#define ENABLE_PIN_2 13


#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

//String input = "";
//char divChar = ',';
//char endChar = '-';
int newpos1 = 0;
int newpos2 = 0;


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
  Serial.begin(19200);
  //Serial.setTimeout(50);
  Serial.flush();

  setupMicrostepping();

  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW);

  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW);

  // Initialize position
  resetPosition();

  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(400);    //2*200
  stepper2.setMaxSpeed(400);
  //stepper1.setSpeed(200);
  stepper1.setAcceleration(1000); //2*500
  stepper2.setAcceleration(1000);

  //Serial.write("Starting!");
}

void loop() {
  // Step the motor with a constant speed as set by setSpeed():
  //stepper2.setSpeed(200);
  //stepper2.runSpeed();

  /*
  // Handle input
  if (Serial.available()){
    //input = Serial.readString();
    input = Serial.readStringUntil(endChar);

    Serial.println("NewPos set to: ");
    Serial.print(input);

    // Dela input i substring
    for (int i = 0; i < input.length(); i++){
      // find divider index and split and convert to int
      if (input[i] == divChar){
        newpos1 = input.substring(0,i).toInt();
        newpos2 = input.substring(i+1,input.length()+1).toInt();
        //Serial.println(i);
      }
    }

    if (newpos1 == 100){
      Serial.write("input1 is 100");
    }
    if (newpos2 == 100){
      Serial.write("input2 is 100");
    }
  } */

  // Handle input method 2
  if (Serial.available()) {
    newpos1 = Serial.parseInt();
    newpos2 = Serial.parseInt();
    int resetPos = Serial.parseInt();
    // Måste skicka 3 ints "100 0 0" annars blir 1 sek delay (ex skicka endast "100")
    
    // Empty serial buffer
    while (Serial.available() > 0) {
      Serial.read();  // Read and discard remaining characters
    }

    /*
    Serial.print("Motor 1: ");
    Serial.println(newpos1);
    Serial.print("Motor 2: ");
    Serial.println(newpos2);
    */

    // Sets the CURRENT position to 0,0 (and moves after this to given steps)
    if (resetPos == 1){
      resetPosition();
      resetPos = 0;
    }
  }

  // Move steppers to position
  if (stepper1.currentPosition() != newpos1 || stepper2.currentPosition() != newpos2){
    stepper1.moveTo(newpos1);
    stepper2.moveTo(newpos2);
    stepper1.run();
    stepper2.run();
    //stepper1.runToPosition();
  }
}