// Include the Arduino Stepper.h library:
#include "Stepper.h"

// Define parameters

// number of steps per rotation:
const int stepsPerRevolution = 2048;

// Motor speed in RPM
const int RPM = 10;

// Motor incremental step
const int STEP = 20;

// Delay time to prevent throttling when reading input from limit switches
const int DELAY_TIME = 10;

// Motor wiring:
// Pin 8 to IN1 on the ULN2003 driver
// Pin 9 to IN2 on the ULN2003 driver
// Pin 10 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver

const int IN1_PIN = 8;
const int IN2_PIN = 9;
const int IN3_PIN = 10;
const int IN4_PIN = 11;

// Pin connected to NC of Limit Switch for CLOSE
const int LIM_CLOSE_PIN = 2;

// Pin connected to NC of Limit Switch for OPEN
const int LIM_OPEN_PIN = 3;

// Note: Common Pin of both Limit switches should be connected to 5V

// variables to store readings of limit switches
int LimCloseRead;
int LimOpenRead;

// Create stepper object
// Note Pin order!!
Stepper pitch_stepper = Stepper(stepsPerRevolution, IN1_PIN, IN3_PIN, IN2_PIN, IN4_PIN);

// define String variable with serial input
String myCmd;

void setup() {

  // declare limit switches pins for input
  pinMode(LIM_OPEN_PIN, INPUT);
  pinMode(LIM_CLOSE_PIN, INPUT);
  
  // bring internal pulldown resistors
  digitalWrite(LIM_OPEN_PIN, LOW);
  digitalWrite(LIM_CLOSE_PIN, LOW);


  // Set the motor speed
  pitch_stepper.setSpeed(RPM);

  // Begin Serial communication at a baud rate of 9600:
  Serial.begin(9600);

    // wait for serial comms
  while (Serial.available() == 0) {
    // pinMode(13, OUTPUT);
  }

}

void loop() {

  // wait for command over Serial
  myCmd = Serial.readStringUntil('\r');


  LimOpenRead = digitalRead(LIM_OPEN_PIN);
  LimCloseRead = digitalRead(LIM_CLOSE_PIN);
  Serial.print("Open Switch is ");
  Serial.print(LimOpenRead);
  Serial.print(" Close Switch is ");
  Serial.println(LimCloseRead);

  if (myCmd == "OPEN") {

    if (digitalRead(LIM_OPEN_PIN) == LOW ) {
      Serial.println("OPEN_SUCCESS");
      // incremental open movement
      pitch_stepper.setSpeed(RPM);
      pitch_stepper.step(STEP);
    } else
      Serial.println("Reached opening limit");
  } else if (myCmd == "CLOSE") {
    if (digitalRead(LIM_CLOSE_PIN) == LOW) {
      Serial.println("CLOSE_SUCCESS");
      // incremental close movement
      pitch_stepper.setSpeed(RPM);
      pitch_stepper.step(-STEP);
    }
    else
      Serial.println("Reached closing limit");
  } else
  { String outputStr;
    outputStr = "Received: [" + myCmd + " ]";
    Serial.println(outputStr);
  }
    // bring internal pulldown resistors
    digitalWrite(LIM_OPEN_PIN, LOW);
    digitalWrite(LIM_CLOSE_PIN, LOW);
    // delay(100);
}
