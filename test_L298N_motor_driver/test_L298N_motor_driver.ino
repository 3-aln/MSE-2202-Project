int motorINT1 = 23;   // control pin for motor
int motorINT2 = 5;    // control pin for motor

void setup() {
  pinMode(motorINT1, OUTPUT);
  pinMode(motorINT2, OUTPUT);
}

void loop() {
  // CLOCKWISE (Looking at the side of the motor with screwheads facing towards you)
  digitalWrite(motorINT1, LOW);
  digitalWrite(motorINT2, HIGH);
  delay(2000);

  // STOP
  digitalWrite(motorINT1, LOW);
  digitalWrite(motorINT2, LOW);
  delay(1000);
  
  // COUNTERCLOCKWISE
  digitalWrite(motorINT1, HIGH);
  digitalWrite(motorINT2, LOW);
  delay(2000);
  
  // STOP
  digitalWrite(motorINT1, LOW);
  digitalWrite(motorINT2, LOW);
  delay(1000);
}
