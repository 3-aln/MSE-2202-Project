// 2021-03-25

// Note: tests performed using Arduino Nano

const int trig_pin = 4;   // HC-SR04 ultrasonic sensor - trigger pin
const int echo_pin = 3;   // HC-SR04 ultrasonic sensor - "echo" pin

long pulse_duration, dist_cm;

void setup() {
  Serial.begin(9600);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
}

void loop() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);

  pulse_duration = pulseIn(echo_pin, HIGH);

  dist_cm = microsecond_to_cm(pulse_duration);

  Serial.print(dist_cm);
  Serial.println(" cm");
}

long microsecond_to_cm(long microseconds) {
  return (microseconds / 29 / 2);
}
