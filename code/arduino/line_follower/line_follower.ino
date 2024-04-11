#include <QTRSensors.h>

// This sketch uses eight RC QTR sensors, connected to digital pins
// SENSOR1 to SENSOR7 as defined below. The sensors' emitter control pin
// (CTRL or LEDON) can optionally be connected to digital pin LEDON,
// or can be left disconnected and remove the call to setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it.
// It prints to the serial monitor a timestamp and the estimated location
// of the line as an integer number between 0 and 7000.
// 0 means the line is directly under sensor 0 or was last seen by sensor 0
// before being lost.
// 1000 means the line is directly under sensor 1,
// 2000 means directly under sensor 2, etc.
// 7000 means the line is directly under sensor 7 or was last seen by sensor 7
// before being lost.

// The commented out section prints also to the serial monitor the sensor values
// as integer numbers from 0 (maximum reflectance) to 1000 (minimum reflectance)

#define LEDON 2
#define SENSOR1 3
#define SENSOR2 4
#define SENSOR3 5
#define SENSOR4 6
#define SENSOR5 7
#define SENSOR6 8
#define SENSOR7 9
#define SENSOR8 10

QTRSensors qtr;

const bool VERBOSE = false;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6, SENSOR7, SENSOR8
  }, SensorCount);

  // qtr.setEmitterPin(LEDON);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // pinMode(LEDON, OUTPUT);
  // digitalWrite(LEDON, HIGH);

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration


  Serial.begin(9600);
  if (VERBOSE)
  {
    // print the calibration minimum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 7000 (for a black line, use readLineBlack(), for white line, use readLineWhite()instead)

  // digitalWrite(LEDON, HIGH);

  uint16_t position = qtr.readLineBlack(sensorValues);

  // digitalWrite(LEDON, LOW);

  Serial.print("E:");
  Serial.print(millis());
  Serial.print('\t');
  Serial.println(position);

  double position_mm;
  char position_mm_char[10];

  if (VERBOSE)
  {
    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print('\t');
    // Print the line position in mm
    position_mm = ((int)position - 3500) * 9.525 / 1000;
    dtostrf(position_mm, 6, 2, position_mm_char);
    Serial.print("Error: ");
    Serial.print(position_mm_char);
    Serial.println(" mm");
  }

  delay(250);
}
