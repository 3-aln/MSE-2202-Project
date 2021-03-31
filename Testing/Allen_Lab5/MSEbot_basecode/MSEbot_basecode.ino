d
//MSE 2202
//Western Engineering base code
//2020 05 13 E J Porter


/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage)
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;
const int ciPB2 = 26;
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciLimitSwitch = 26;
const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 12;   // 12
const int ciMotorRightB = 19;   // 19
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>  // Library used for Smart LEDS
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;    // microseconds
const int CR1_ciHeartbeatInterval = 500;
const int CR1_ciMotorRunTime = 1000;    // default time between inner case 0 actions: 1000 (= 1 second)
const int CR1_ciMotorRunTime_short = 1000; // short pause
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint8_t ci8RightTurn = 22;  // Orig: 18
const uint8_t ci8LeftTurn = 12;

const uint8_t ci8_180Turn = 44;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;
uint8_t test_uint8_t = 100;    // test

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;       // microseconds
unsigned long CR1_ulMainTimerNow_millis;       // milliseconds

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

boolean triggerMvt = true;

// Set up servo motor
const int servoPin = 15;
const int servoChannel = 7;
int servoPos[] = {0, 90, 90, 180, 0, 180, 0, 180, 90, 90, 0};
int servo_index = 0;
int servo_wait = 500;
boolean servo_active = false;
unsigned long servo_time_prev;



// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setup() {
  Serial.begin(115200);
  Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud

  Core_ZEROInit();

  WDT_EnableFastWatchDogCore1();
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[0] = 0;
  WDT_vfFastWDTWarningCore1[1] = 0;
  WDT_vfFastWDTWarningCore1[2] = 0;
  WDT_vfFastWDTWarningCore1[3] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[4] = 0;
  WDT_vfFastWDTWarningCore1[5] = 0;
  WDT_vfFastWDTWarningCore1[6] = 0;
  WDT_vfFastWDTWarningCore1[7] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[8] = 0;
  WDT_vfFastWDTWarningCore1[9] = 0;
  WDT_ResetCore1();

  setupMotion();
  pinMode(ciHeartbeatLED, OUTPUT);
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(ciLimitSwitch, INPUT_PULLUP);

  SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
  SmartLEDs.clear();                          // Set all pixel colours to off
  SmartLEDs.show();                           // Send the updated pixel colours to the hardware

  // Set up servo
  ledcAttachPin(servoPin, servoChannel);
  ledcSetup(servoChannel, 50, 16);
}

void loop()
{
  //WSVR_BreakPoint(1);

  //average the encoder tick times
  ENC_Averaging();     // Fn from `Encoder.h` - used to determine speed.

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
    CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
      iButtonState = iButtonValue;               // update current button state

      // only toggle the run condition if the new button state is LOW
      if (iButtonState == LOW)
      {
        ENC_ClearLeftOdometer();     // Clear odometers every time the robot is started and stopped (button push)
        ENC_ClearRightOdometer();    //
        btRun = !btRun;
        Serial.println(btRun);
        Serial.println((test_uint8_t * 0.9512));
        Serial.println((uint8_t)floor(test_uint8_t * 0.9512));
        // if stopping, reset motor states and stop motors
        if (!btRun)
        {
          ucMotorStateIndex = 0;
          ucMotorState = 0;
          move(0);
        }

        if (!triggerMvt) {
          triggerMvt = true;
        }
      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state

  // @@ LIMIT SWITCH
  if (!digitalRead(ciLimitSwitch))
  {
    btRun = 0; //if limit switch is pressed stop bot
    ucMotorStateIndex = 0;
    ucMotorState = 0;
    move(0);
  }


  // @@ READING THE BEACON CHARACTERS
  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
    // Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
  }


  // @@ MAIN TIMER
  CR1_ulMainTimerNow = micros();
  CR1_ulMainTimerNow_millis = millis();
  if (CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer) // go through loop every 1 ms
  {
    //    Serial.println(CR1_ui8WheelSpeed);

    WDT_ResetCore1();    // reset watchdog timer - don't really need to worry about this.
    WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;

    CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

    switch (CR1_ucMainTimerCaseCore1) //full switch run through is 1mS
    {
      //###############################################################################
      // @@ MAIN MOTOR RUN CODE
      // Enter `case 0` to indicate the desired task to perform.

      case -1:   // LEVEL 1 TASK -------------------------------------------------------
        {

          if (btRun)
          {
            CR1_ulMotorTimerNow = millis();

            switch (ucMotorStateIndex) {
              // MOVE FORWARD
              case 0: {
                  if (triggerMvt) {
                    ENC_SetDistance(848, 848);  // 837 + 11
                    ucMotorState = 1;   //forward
                    //            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    //            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.980);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex = 1;
                    triggerMvt = true;
                  }
                  break;
                }

              // PAUSE
              case 1: {
                  ucMotorStateIndex = 1;
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime) {   // stay still for 1 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex = 2;
                  }
                  break;
                }
              // REVERSE HALFWAY
              case 2: {
                  if (triggerMvt) {
                    ucMotorState = 4;  //reverse
                    ENC_SetDistance(-424, -424);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.965);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex = 3;
                  }
                  break;
                }
              // PAUSE
              case 3: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime) {   // stay still for 1 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex = 4;
                    triggerMvt = true;
                  }
                  break;
                }
              // 180 DEGREE TURN
              case 4: {
                  if (triggerMvt) {
                    ENC_SetDistance(ci8_180Turn, -(ci8_180Turn));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorState = 3;  //
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex =  5;
                  }

                  break;
                }
              // PAUSE BEFORE FLAG
              case 5: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= 250) {   // stay still for 0.25 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex = 12;
                    triggerMvt = false;
                    servo_active = true;
                    servo_index = 0;
                  }
                  break;
                }
            }
          }
          CR1_ucMainTimerCaseCore1 = 1;

          break;
        }

      case 0:   // LEVEL 2 TASK -------------------------------------------------------
        {

          if (btRun)
          {
            CR1_ulMotorTimerNow = millis();

            switch (ucMotorStateIndex) {
              // FIRST PAUSE
              case 0: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                  }
                  break;
                }
              // MOVE FORWARD
              case 1: {
                  if (triggerMvt) {
                    ENC_SetDistance(220, 220);
                    ucMotorState = 1;   //forward
                    //            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    //            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.980);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }

              // PAUSE
              case 2: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // TURN RIGHT
              case 3: {
                  if (triggerMvt) {
                    ucMotorState = 3;  // right
                    ENC_SetDistance(ci8RightTurn - 4, -(ci8RightTurn - 4));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                  }
                  break;
                }
              // PAUSE
              case 4: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // MOVE FORWARD
              case 5: {
                  if (triggerMvt) {
                    ENC_SetDistance(165, 165);
                    ucMotorState = 1;   //forward
                    //            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    //            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.980);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }

              // PAUSE
              case 6: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // TURN LEFT
              case 7: {
                  if (triggerMvt) {
                    ENC_SetDistance(-(ci8LeftTurn + 5), ci8LeftTurn + 5);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorState = 2;  // left
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                  }

                  break;
                }
              // PAUSE
              case 8: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // MOVE FORWARD
              case 9: {
                  if (triggerMvt) {
                    ENC_SetDistance(175, 175);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.980);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // PAUSE
              case 10: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // TURN LEFT
              case 11: {
                  if (triggerMvt) {
                    ENC_SetDistance(-(ci8LeftTurn + 3), ci8LeftTurn + 3);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorState = 2;  // left
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                  }

                  break;
                }
              // PAUSE
              case 12: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // MOVE FORWARD
              case 13: {
                  if (triggerMvt) {
                    ENC_SetDistance(165, 165);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.980);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // PAUSE
              case 14: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // TURN RIGHT
              case 15: {
                  if (triggerMvt) {
                    ucMotorState = 3;  // right
                    ENC_SetDistance(ci8RightTurn - 3, -(ci8RightTurn - 3));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                  }
                  break;
                }
              // PAUSE
              case 16: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // MOVE FORWARD
              case 17: {
                  if (triggerMvt) {
                    ENC_SetDistance(275, 275);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.980);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // PAUSE
              case 18: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime_short) {   // stay still for 0.5 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // REVERSE HALFWAY
              case 19: {
                  if (triggerMvt) {
                    ucMotorState = 4;  //reverse
                    ENC_SetDistance(-350, -350);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = (uint8_t)floor(CR1_ui8WheelSpeed * 0.965);
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                  }
                  break;
                }
              // PAUSE
              case 20: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime) {   // stay still for 1 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex++;
                    triggerMvt = true;
                  }
                  break;
                }
              // 180 DEGREE TURN
              case 21: {
                  if (triggerMvt) {
                    ENC_SetDistance(ci8_180Turn, -(ci8_180Turn));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorState = 3;  //
                    triggerMvt = false;
                  }

                  if (!ENC_ISMotorRunning()) {
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow; // update previous motor timer for the next case.
                    ucMotorStateIndex++;
                  }

                  break;
                }
              // PAUSE BEFORE FLAG
              case 22: {
                  ucMotorState = 0;
                  move(0);
                  if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= 250) {   // stay still for 0.25 s
                    CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
                    ucMotorStateIndex = 99;
                    triggerMvt = false;
                    servo_active = true;
                    servo_index = 0;
                  }
                  break;
                }

            }
          }
          CR1_ucMainTimerCaseCore1 = 1;

          break;
        }
      //###############################################################################
      case 1:
        {
          //read pot 1 for motor speeds
          CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion

          CR1_ucMainTimerCaseCore1 = 2;
          break;
        }
      //###############################################################################
      case 2:
        {
          // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS

          //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS

          CR1_ucMainTimerCaseCore1 = 3;
          break;
        }
      //###############################################################################
      case 3:
        {
          //move bot X number of odometer ticks
          if (ENC_ISMotorRunning())   // from `Encoder.h`
          {
            MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed, CR1_ui8RightWheelSpeed);
          }

          CR1_ucMainTimerCaseCore1 = 4;
          break;
        }
      //###############################################################################
      case 4:
        {
          if (servo_active) {
            if (CR1_ulMainTimerNow_millis - servo_time_prev >= servo_wait) {
              servo_time_prev = CR1_ulMainTimerNow_millis;

              ledcWrite(servoChannel, degreesToDutyCycle(servoPos[servo_index]));
              servo_index++;
            }
            if (servo_index > 10) {
              servo_active = false;
            }
          }

          CR1_ucMainTimerCaseCore1 = 5;
          break;
        }
      //###############################################################################
      case 5:
        {


          CR1_ucMainTimerCaseCore1 = 6;
          break;
        }
      //###############################################################################
      case 6:
        {


          CR1_ucMainTimerCaseCore1 = 7;
          break;
        }
      //###############################################################################
      case 7:
        {
          // @@ SET SMART LED COLOURS
          if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
            SmartLEDs.setPixelColor(0, 0, 25, 0);      // make LED1 green with 10% intensity
          }
          else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
            SmartLEDs.setPixelColor(0, 25, 0, 25);     // make LED1 purple with 10% intensity
          }
          else {                                       // otherwise
            SmartLEDs.setPixelColor(0, 25, 0, 0);      // make LED1 red with 10% intensity
          }
          SmartLEDs.show();                            // send updated colour to LEDs

          CR1_ucMainTimerCaseCore1 = 8;
          break;
        }
      //###############################################################################
      case 8:
        {

          CR1_ucMainTimerCaseCore1 = 9;
          break;
        }
      //###############################################################################
      case 9:
        {

          CR1_ucMainTimerCaseCore1 = 0;
          break;
        }

    }
  }

  // Heartbeat LED
  CR1_ulHeartbeatTimerNow = millis();
  if (CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
  {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
    // Serial.println((vui32test2 - vui32test1)* 3 );
  }

}

long degreesToDutyCycle(int deg) {
  const long minDutyCycle = 1675;
  const long maxDutyCycle = 8050;

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);

  return dutyCycle;
}
