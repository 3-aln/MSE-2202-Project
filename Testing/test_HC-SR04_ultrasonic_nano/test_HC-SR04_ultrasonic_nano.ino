// 2021-03-25

// Note: tests performed using Arduino Nano

const int trig_pin = 4;   // HC-SR04 ultrasonic sensor - trigger pin
const int echo_pin = 3;   // HC-SR04 ultrasonic sensor - "echo" pin

long pulse_duration, dist_cm;
const int short_pulse = 2;  // microseconds
const int long_pulse = short_pulse + 10; // microseconds

unsigned long us_now;     // store current operation time in microseconds
unsigned long us_prev_short_pulse;  // previous short pulse time
unsigned long us_prev_long_pulse;   // previous long pulse time

void setup() {
  Serial.begin(9600);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
}

void loop() {
  us_now = micros();

  if (us_now - us_prev_short_pulse >= short_pulse) {
    us_prev_short_pulse = us_now;
    digitalWrite(trig_pin, HIGH);
  }

  if (us_now - us_prev_long_pulse >= long_pulse) {
    digitalWrite(trig_pin, LOW);

    pulse_duration = pulseIn(echo_pin, HIGH);

    dist_cm = microsecond_to_cm(pulse_duration);
    Serial.print(dist_cm);
    Serial.println(" cm");
  }


  
//  digitalWrite(trig_pin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trig_pin, HIGH);
//  delayMicroseconds(10);
//
//  pulse_duration = pulseIn(echo_pin, HIGH);
//
//  dist_cm = microsecond_to_cm(pulse_duration);
//
//  Serial.print(dist_cm);
//  Serial.println(" cm");
}

long microsecond_to_cm(long microseconds) {
  return (microseconds / 29 / 2);
}
