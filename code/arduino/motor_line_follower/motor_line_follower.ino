// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// Line follower connections
const int pinRotate = A0;
const int RotateMin = 0;
const int RotateMax = 1023;
const int pinSpeed = A1;
const int SpeedMin = 0;
const int SpeedMax = 1023;


void setup() {

  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {

  //read values
  int RotateValue = getRotateValue();
  int SpeedValue = getSpeedValue();

  //compute motorA, motorB and fwdA, fwdB from input RotateValue, SpeedValue
  bool fwdA = true;
  bool fwdB = true;
  int motorA = map(SpeedValue, SpeedMin, SpeedMax, 255, -255) +
               map(RotateValue, RotateMin, RotateMax, -255, 255);
  int motorB = map(SpeedValue, SpeedMin, SpeedMax, 255, -255) -
               map(RotateValue, RotateMin, RotateMax, -255, 255);

  if (motorA < 0)
  {
    fwdA = false;
    motorA = - motorA;
  }

  if (motorB < 0)
  {
    fwdB = false;
    motorB = - motorB;
  }

  if (motorA > 255) motorA = 255;
  if (motorB > 255) motorB = 255;

  // avoid humming
  if (motorA < 15) motorA = 0;
  if (motorB < 15) motorB = 0;


  showValuesSerial(RotateValue, SpeedValue, motorA, fwdA, motorB, fwdB);

  operateMotors(motorA, fwdA, motorB, fwdB, in1, in2, in3, in4, enA, enB);

  if (fwdA == true)
  {
    // move motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    // move motor A in reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if (fwdB == true)
  {
    // move motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    // move motor B in reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  analogWrite(enA, motorA);
  analogWrite(enB, motorB);

  delay(100);
}

int getRotateValue()
{

  int result = 0;
  delay(100);   // short delay needed between analog readings
  result = analogRead(pinRotate);
  return result;
}

int getSpeedValue()
{
  int result = 0;
  delay(100);   // short delay needed between analog readings
  result = analogRead(pinSpeed);
  return result;
}

void showValuesSerial(int RotateValue, int SpeedValue,
                      int motorA, bool fwdA,
                      int motorB, bool fwdB)
{
  Serial.print("(X, Y): (");
  Serial.print(RotateValue);
  Serial.print(", ");
  Serial.print(SpeedValue);
  Serial.print(") | motorA: ");
  if (fwdA == false) {
    Serial.print("-");
  }
  Serial.print(map(motorA, 0, 255, 0, 100));
  Serial.print("% | motorB: ");
  if (fwdB == false) {
    Serial.print("-");
  }
  Serial.print(map(motorB, 0, 255, 0, 100));
  Serial.println("%");
}

void operateMotors(int motorA, bool fwdA, int motorB, bool fwdB,
                   int in1, int in2, int in3, int in4, int enA, int enB)
{

  if (fwdA == true)
  {
    // move motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    // move motor A in reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if (fwdB == true)
  {
    // move motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    // move motor B in reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  analogWrite(enA, motorA);
  analogWrite(enB, motorB);

}
