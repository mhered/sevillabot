// This code controls a stepper for aiming, a laser target and the firing mechanism of the gun


// Arduino Stepper library
#include "Stepper.h"

// Trigger
// Pin3 to Relay IN
const int relayPin = 3    ;

const int delayTime = 100;

// trigger position
int triggerRead;

// Laser:
// Pin5 to Laser
const int laserPin = 5;

// Define number of steps per rotation:
const int stepsPerRevolution = 2048;
const int stepperRPM = 1;


// Wiring of the ULN2003 stepper driver:
// Pin 8 to IN1
// Pin 9 to IN2
// Pin 10 to IN3
// Pin 11 to IN4

const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;


// Create stepper object called 'aimingStepper', note the pin order:
Stepper aimingStepper = Stepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// define String variable for serial input
String myCmd;

void setup() {
  // Set the stepper speed (in rpm)
  aimingStepper.setSpeed(stepperRPM);

  /*
    // Configure limit switch Pin
    pinMode(triggerPin, INPUT);
    // hack to bring the internal pullup resistor
    digitalWrite(triggerPin, HIGH);
  */

  pinMode(relayPin, OUTPUT);
  pinMode(laserPin, OUTPUT);


  // Begin Serial communication at a baud rate of 9600:
  Serial.begin(9600);

}

void loop() {
  while (Serial.available() == 0) {
    // switch on Arduino LED while comms are not yet up
    pinMode(13, OUTPUT);
  }

  // get command from python program
  myCmd = Serial.readStringUntil('\r');
  // Serial.print("Command ");
  // Serial.println(myCmd);

  if (myCmd == "UP") {
    // Step 1/128 revolution in one direction:
    aimingStepper.setSpeed(stepperRPM);
    aimingStepper.step(3);
    Serial.println("UP_SUCCESS");
  } else if (myCmd == "DOWN") {
    aimingStepper.setSpeed(stepperRPM);
    aimingStepper.step(-3);
    Serial.println("DOWN_SUCCESS");

  } else if (myCmd == "FIRE") {
    // Send a pulse
    digitalWrite(relayPin, HIGH);
    delay(delayTime);
    digitalWrite(relayPin, LOW);
    delay(delayTime);
    Serial.println("FIRE_SUCCESS");

  } else if (myCmd == "LASER") {
    digitalWrite(laserPin, !digitalRead(laserPin));
    Serial.println("LASER_SUCCESS");
  } else
  { Serial.println("unknown");

  }


}
