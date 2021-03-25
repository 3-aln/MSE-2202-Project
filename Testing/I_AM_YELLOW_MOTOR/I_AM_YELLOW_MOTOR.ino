  
//Switch to turn on and off. We will not include this switch since the car is autonomous.
  const int sideSwitch = 13;

//Motor pin and channel
  int yellowMotorPin = 15;
  int motorChannel = 7;

  //LED pins!
  int LED1 = 2;
  int smartyLED = 25;


//Initial switch state
  int sideSwitchState = LOW;

  //NeoPixel is for the colored LEDs
  #include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);

void setup() {

  pinMode(sideSwitch, INPUT);
  pinMode(yellowMotorPin, OUTPUT);

  //Channel, Hz, bit
  ledcSetup(motorChannel, 500, 8);
  ledcAttachPin(yellowMotorPin, motorChannel);

   SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
   SmartLEDs.clear();                          // Set all pixel colours to off
   SmartLEDs.show();                           // Send the updated pixel colours to the hardware

}

void loop() {
  // put your main code here, to run repeatedly:
  sideSwitchState = digitalRead(sideSwitch);

  //Runs the motor.
  if(sideSwitchState == HIGH){

    //channel, duty cycle
    ledcWrite(motorChannel, 250);
    SmartLEDs.setPixelColor(0,25,0,25);

    //KEY PART: from the multimeter, I/O pin at 15 to GND showed about 1.3V, which is good to open the gate for the VN3205 N-MOSFET
    digitalWrite(yellowMotorPin, HIGH);
  }

  //Stop the motor.
  else if(sideSwitchState == LOW){
    
    //channel, duty cycle
    ledcWrite(motorChannel, 0);
    SmartLEDs.setPixelColor(0,0,25,0); 

    
    digitalWrite(yellowMotorPin, LOW);
  }

//Shows the currently set color.
  SmartLEDs.show();


}
