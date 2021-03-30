#include <NewPing.h>

// Testing functionality of HC-SR04 ultrasonic sensor module using Arduino Nano.
// Uses NewPing library to simplify code and improve reading quality.

const int trig_pin = 4;         // HC-SR04 ultrasonic sensor - trigger pin
const int echo_pin = 3;         // HC-SR04 ultrasonic sensor - "echo" pin
const int MAX_DISTANCE = 400;   // ultrasonic sensor is rated for around 400 cm

unsigned long ms_now;               // store current operation time in milliseconds
unsigned long ms_prev_ping;         // previous pulse in milliseconds

float distance_cm;        // detected distance using ultrasonic sensor in cm

NewPing sonar(trig_pin, echo_pin, MAX_DISTANCE);  // initialize ultrasonic device

void setup() {
  Serial.begin(9600);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
}

void loop() {
  ms_now = millis();

  // Every 50 ms, send ping and display distance.
  if (ms_now - ms_prev_ping >= ping_interval) {
    distance_cm = sonar.ping_cm();
    Serial.print(distance_cm);
    Serial.println(" cm");
  }
}
