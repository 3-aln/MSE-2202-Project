#include <NewPing.h>

// Testing functionality of HC-SR04 ultrasonic sensor module with the ESP32.
// Uses NewPing library to simplify code and improve reading quality.
// Requires voltage divider to reduce output from echo pin from 5 V to ~3.3 V.

// ------------------------------------------------
const int US_trig_pin = 21;
const int US_echo_pin = 33;

const int motorPin = 5;
// ------------------------------------------------

// ------------------------------------------------------------------------------------------------
const int MAX_DISTANCE = 400;   // ultrasonic sensor is rated for around 400 cm
NewPing sonar(US_trig_pin, US_echo_pin, MAX_DISTANCE);    // initialize ultrasonic device
unsigned int US_distance_cm;           // detected distance using ultrasonic sensor in cm
// ------------------------------------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  digitalWrite(motorPin, LOW);

  US_distance_cm = sonar.convert_cm(sonar.ping_median(5));

  Serial.print(US_distance_cm);
  Serial.println(" cm");
}
