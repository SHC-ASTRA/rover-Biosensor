/**
 * @file Template.cpp
 * @author your name (you@domain.com)
 * @brief description
 *
 */

//------------//
//  Includes  //
//------------//

// #include "AstraMisc.h"
// #include "project/TEMPLATE.h"
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <LSS.h>
#include <vector>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include "AstraVicCAN.h"

//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1
#define LSS_ID (3)
#define LSS_BAUD (LSS_DefaultBaud)
#define LSS_SERIAL (Serial)
#define LED_PIN 13 // Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)
#define CAN_TX 22
#define CAN_RX 20

//---------------------//
//  Component classes  //
//---------------------//
// There is only one smart servo for Citadel
LSS myLSS = LSS(LSS_ID);

hw_timer_t *Timer0_Cfg = NULL, *Timer1_Cfg = NULL;
Servo servo1, servo2, servo3;

//----------//
//  Timing  //
//----------//

bool ledState = false;
unsigned long fanTimer, fansTimer, fanTimer_1, fanTimer_2, fanTimer_3;
unsigned long pumpTimer, pumpsTimer, pumpTimer_1, pumpTimer_2, pumpTimer_3;
bool fansOn = 0, fanOn_1 = 0, fanOn_2 = 0, fanOn_3 = 0; // 1 = long = 2seconds, 0 = short = 0.5s
bool pumpON = 0, pumpON_1 = 0, pumpON_2 = 0, pumpON_3 = 0;
unsigned long prevFanTime = 0, prevFanTime_1 = 0, prevFanTime_2 = 0, prevFanTime_3 = 0;

//--------------//
//  Prototypes  //
//--------------//
void activateCapSer(int num, int truFal);
void initialize();
std::vector<String> parseInput(String input, const char delim);

void IRAM_ATTR Timer0_ISR()
{
  // portENTER_CRITICAL_ISR(&timer0Mux);
  if (fansOn && (millis() - prevFanTime >= fansTimer))
  {
    digitalWrite(14, LOW);
    digitalWrite(32, LOW);
    digitalWrite(15, LOW);
    fansOn = 0;
    fansTimer = 0;
  }
  else if (fanOn_1 && (millis() - prevFanTime_1 >= fanTimer_1))
  {
    digitalWrite(14, LOW);
    fanOn_1 = 0;
    fanTimer_1 = 0;
  }
  else if (fanOn_2 && (millis() - prevFanTime_2 >= fanTimer_2))
  {
    digitalWrite(32, LOW);
    fanOn_2 = 0;
    fanTimer_2 = 0;
  }
  else if (fanOn_3 && (millis() - prevFanTime_3 >= fanTimer_3))
  {
    digitalWrite(15, LOW);
    fanOn_3 = 0;
    fanTimer_3 = 0;
  }
  // portEXIT_CRITICAL_ISR(&timer0Mux);
}

void IRAM_ATTR Timer1_ISR()
{
  digitalWrite(LED_BUILTIN, !ledState);
  ledState = !ledState;
}

// }
// std::vector<String> parseInput(String input, const char delim);

//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup()
{
  initialize();
}

//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//
void loop()
{
  //----------//
  //  Timers  //
  //----------//

  //-------------//
  //  CAN input  //
  //-------------//

  // Send serial command thorugh the onbaord computer to rover core microcontroller to other microcontrollers and read serial commmands.

  //------------------//
  //  UART/USB input  //
  //------------------//
  //
  //
  //-------------------------------------------------------//
  //                                                       //
  //      /////////    //\\        ////    //////////      //
  //    //             //  \\    //  //    //        //    //
  //    //             //    \\//    //    //        //    //
  //    //             //            //    //        //    //
  //    //             //            //    //        //    //
  //    //             //            //    //        //    //
  //      /////////    //            //    //////////      //
  //                                                       //
  //-------------------------------------------------------//
  // if (vicCAN.readCan())
  // {
  //   // CAN command handling goes here... can be copied from Core or Digit
  //   const uint8_t commandID = vicCAN.getCmdId();
  //   static std::vector<double> canData;
  //   vicCAN.parseData(canData);

  //   Serial.print("VicCAN: ");
  //   Serial.print(commandID);
  //   Serial.print("; ");
  //   if (canData.size() > 0)
  //   {
  //     for (const double &data : canData)
  //     {
  //       Serial.print(data);
  //       Serial.print(", ");
  //     }
  //   }
  //   Serial.println();

  //   /**/ if (commandID == CMD_PING)
  //   {
  //     vicCAN.respond(1); // "pong"
  //     Serial.println("Received ping over CAN");
  //   }
  //   else if (commandID == CMD_LSS_TURNBY_DEG)
  //   {
  //     if (canData.size() == 1)
  //     {
  //     }
  //   }
  //   else if (commandID == CMD_PWMSERVO_SET_DEG)
  //   {
  //     if (canData.size() == 2)
  //     {
  //       switch (int(canData[0]))
  //       {
  //       case 1:
  //         servo1.write(int(canData[1]));
  //         break;
  //       case 2:
  //         servo2.write(int(canData[1]));
  //         break;
  //       case 3:
  //         servo2.write(int(canData[1]));
  //         break;
  //       default:
  //         break;
  //       }
  //     }
  //   }
  //   else if (commandID == CMD_LSS_TURNBY_DEG)
  //   {

  //     if (canData.size() == 1)
  //     {
  //       switch (int(canData[0]))
  //       {
  //       case -900:
  //         myLSS.move(-900);
  //         Serial.println("Full Retractig CITADEL arm");
  //         break;
  //       case 0:
  //         myLSS.move(0);
  //         Serial.println("Setting arm to half extend");
  //         break;
  //       case 20:
  //         myLSS.moveRelative(20);
  //         Serial.println("Extending CITADEL arm");
  //         break;
  //       case -20:
  //         myLSS.moveRelative(-20);
  //         Serial.println("Extending CITADEL arm");
  //         break;
  //       case 2:
  //         myLSS.reset();
  //         Serial.println("Servo Reset");
  //         break;
  //       default:
  //         break;
  //       }
  //     }
  //     else
  //     {
  //       switch (int(canData[0]))
  //       {
  //       case 0:
  //         myLSS.moveRelative(int(canData[1]) * 10);
  //         Serial.println(myLSS.getPosition());
  //         break;
  //       default:
  //         break;
  //       }
  //     }
  //     // if (parCmd[1] == "Relative")
  //     // {
  //     //   myLSS.moveRelative(((parCmd[2]).toInt()) * 10);
  //     //   Serial.println(myLSS.getPosition());
  //     // }
  //     // else if (parCmd[1] == "FullRetract")
  //     // {
  //     //   myLSS.moveRelative(((parCmd[2]).toInt()) * 10);
  //     //   Serial.println(myLSS.getPosition());
  //     // }
  //     // else if (parCmd[1] == "FullRetract")
  //     // {
  //     //   myLSS.move(-900);
  //     //   Serial.println("Full Retractig CITADEL arm");
  //     // }
  //     // else if (parCmd[1] == "Half")
  //     // {
  //     //   myLSS.move(0);
  //     //   Serial.println("Setting arm to half extend");
  //     // }
  //     // else if (parCmd[1] == "Extend")
  //     // { //
  //     //   myLSS.moveRelative(20);
  //     //   Serial.println("Extending CITADEL arm");
  //     // }
  //     // else if (parCmd[1] == "Retract")
  //     // { //
  //     //   myLSS.moveRelative(-20);
  //     //   Serial.println("Retractig CITADEL arm");
  //     // }
  //     // else if (parCmd[1] == "Reset")
  //     // {
  //     //   myLSS.reset();
  //     //   Serial.println("Servo Reset");
  //     // }
  //   }
  //   // else if (commandID == CMD_STEPPER_CTRL)
  //   // {
  //   //   switch(int(canData[0]))
  //   //   {
  //   //     case 0:

  //   //   }
  //   // }
  //   else if (commandID == CMD_DCMOTOR_CTRL)
  //   {
  //     if (bool(canData[1]))
  //       digitalWrite(25, HIGH);
  //     else
  //       digitalWrite(25, LOW);
  //   }
  // }
  // ...

  // Within Serial command handling:
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    Serial.println(input);

    input.trim();                    // Remove preceding and trailing whitespace
    std::vector<String> parCmd = {}; // Initialize empty vector to hold separated arguments
    parCmd = parseInput(input, ','); // Separate `input` by commas and place into parCmd vector
    parCmd[0].toLowerCase();         // Make command case-insensitive
    String command = parCmd[0];      // To make processing code more readable

    //--------//
    //  Misc  //
    //--------//
    /**/ if (command == "ping")
    {
      Serial.println("pong");
    }

    else if (command == "time")
    {
      Serial.println(millis());
    }
    else if (parCmd[0] == "can_relay_tovic")
    {
      vicCAN.relayFromSerial(parCmd);
    }
    else if (parCmd[0] == "can_relay_mode")
    {
      if (parCmd[1] == "on")
      {
        vicCAN.relayOn();
      }
      else if (parCmd[1] == "off")
      {
        vicCAN.relayOff();
      }
    }

    else if (command == "led")
    {
      digitalWrite(LED_BUILTIN, !ledState);
      ledState = !ledState;
    }

    //-----------//
    //  Sensors  //
    //-----------//

    //----------//
    //  Motors  //
    //----------//
    // Fan
    if (parCmd[0] == "fans")
    { // Is looking for a command that looks like "fan,0,0,0,time"

      Serial.println(parCmd[0]);
      digitalWrite(14, parCmd[2].toInt());
      digitalWrite(15, parCmd[2].toInt());
      digitalWrite(15, HIGH);
      fansOn = 1;
      fansTimer = parCmd[1].toInt();
      prevFanTime = millis();
      Serial.println("Fans Activated");
      // Pump
    }
    else if (parCmd[0] == "fan")
    {
      // if (command != input)
      // {
      Serial.println("Reached fan");
      switch (parCmd[1].toInt())
      {
      case 1:
        digitalWrite(14, parCmd[2].toInt());
        fanOn_1 = 1;
        fanTimer_1 = parCmd[3].toInt();
        prevFanTime_1 = millis();
        break;
      case 2:
        digitalWrite(32, parCmd[2].toInt());
        fanOn_2 = 1;
        fanTimer_2 = parCmd[3].toInt();
        prevFanTime_2 = millis();
        break;
      case 3:
        digitalWrite(15, parCmd[2].toInt());
        fanOn_3 = 1;
        fanTimer_3 = parCmd[3].toInt();
        prevFanTime_3 = millis();
        break;
      default:
        break;
      }
      Serial.println("Fan Activated");
    }
    // else if (parCmd[0] == "pump")
    // { // Is looking for a command that looks like "pump,0,0,0,time"
    //   if (command != input)
    //   {
    //     input = command;
    //     digitalWrite(38 + parCmd[1].toInt(), parCmd[2].toInt());
    //     pumpON = 1;
    //     pumpTimer = millis() + parCmd[4].toInt();
    //     Serial.println("Pumps Activated");
    //   }
    //   // Servo
    // }
    // else if (parCmd[0] == "pumps")
    // {
    //   if (command != input)
    //   {
    //     input = command;
    //     for (int i = 38; i < 42; i++)
    //     {
    //       digitalWrite(38 + parCmd[1].toInt(), parCmd[2].toInt());
    //     }
    //     pumpON = 1;
    //     pumpTimer = millis() + parCmd[4].toInt();
    //     Serial.println("Pumps Activated");
    //   }
    // }

    // else if (parCmd[0] == "Smartservo")
    // {
    //   if (command != input)
    //   {
    //     if (parCmd[1] == "Relative")
    //     {
    //       myLSS.moveRelative(((parCmd[2]).toInt()) * 10);
    //       Serial.println(myLSS.getPosition());
    //     }
    //     else if (parCmd[1] == "FullRetract")
    //     {
    //       myLSS.moveRelative(((parCmd[2]).toInt()) * 10);
    //       Serial.println(myLSS.getPosition());
    //     }
    //     else if (parCmd[1] == "FullRetract")
    //     {
    //       myLSS.move(-900);
    //       Serial.println("Full Retractig CITADEL arm");
    //     }
    //     else if (parCmd[1] == "Half")
    //     {
    //       myLSS.move(0);
    //       Serial.println("Setting arm to half extend");
    //     }
    //     else if (parCmd[1] == "Extend")
    //     { //
    //       myLSS.moveRelative(20);
    //       Serial.println("Extending CITADEL arm");
    //     }
    //     else if (parCmd[1] == "Retract")
    //     { //
    //       myLSS.moveRelative(-20);
    //       Serial.println("Retractig CITADEL arm");
    //     }
    //     else if (parCmd[1] == "Reset")
    //     {
    //       myLSS.reset();
    //       Serial.println("Servo Reset");
    //     }
    //   }
    // }
    // Hardcode limits for servos
    else if (parCmd[0] == "servo")
    {
      switch (parCmd[1].toInt())
      {
      case 1:
        servo1.write(parCmd[2].toInt());
        break;
      case 2:
        servo2.write(parCmd[2].toInt());
        break;
      case 3:
        servo3.write(parCmd[2].toInt());
        break;
      default:
        break;
      }
    }

    // else if (parCmd[0] == "Stepper")
    // {
    //   Serial1.print(input);
    // }

    else if (parCmd[0] == "shutdown")
    {
      digitalWrite(13, LOW);
      digitalWrite(14, LOW);
      digitalWrite(15, LOW);
      digitalWrite(32, LOW);
      digitalWrite(33, LOW);
      digitalWrite(5, LOW);
      digitalWrite(12, LOW);
      digitalWrite(19, LOW);
      digitalWrite(21, LOW);
      digitalWrite(27, LOW);

      myLSS.reset();
      servo1.detach();
      servo2.detach();
      servo3.detach();
    }
  }
}

//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//

void initialize()
{
  //--------//
  //  Pins  //
  //--------//

  // ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX);
  // Serial.println("CAN has started");
  Serial.begin(115200);
  // Serial1.begin(115200);
  while (!Serial)
    ;
  Serial.println("Serial has started");
  pinMode(7, OUTPUT); // Pi Tx   (UART) // UART
  pinMode(8, INPUT);  // Pi Rx   (UART) // UART
  Serial.println("UART has started");
  pinMode(LED_PIN, OUTPUT);
  // while (!Serial1)
  //   ;

  // Fans
  pinMode(14, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(15, OUTPUT);

  // Vibrator
  pinMode(13, OUTPUT);

  // Pumps
  // pinMode(38, OUTPUT);
  // pinMode(39, OUTPUT);
  // pinMode(40, OUTPUT);
  // pinMode(41, OUTPUT);

  digitalWrite(14, LOW); // Fan 1
  digitalWrite(32, LOW); // Fan 2
  digitalWrite(15, LOW); // Fan 3
  //
  digitalWrite(13, LOW); // Vibrator

  // digitalWrite(20, LOW); // Pump 1
  // digitalWrite(22, LOW); // Pump 2
  // digitalWrite(23, LOW); // Pump 3
  // digitalWrite(17, LOW); // Pump 4

  //------------------//
  //  Communications  //
  //------------------//

  LSS::initBus(LSS_SERIAL, LSS_BAUD);
  delay(2000);
  myLSS.setAngularStiffness(0);
  myLSS.setAngularHoldingStiffness(0);
  myLSS.setAngularAcceleration(15);
  myLSS.setAngularDeceleration(15);

  Serial.println("Smart servo has started");

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 5000, true);
  timerAlarmEnable(Timer0_Cfg);

  Timer1_Cfg = timerBegin(1, 80, true);
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, 100000, true);
  timerAlarmEnable(Timer1_Cfg);

  Serial.println("timers has started");

  //------------------//
  //  Servos          //
  //------------------//

  servo1.attach(5);
  servo2.attach(19);
  servo3.attach(21);

  //-----------//
  //  Sensors  //
  //-----------//

  //--------------------//
  //  Misc. Components  //
  //--------------------//
}

std::vector<String> parseInput(String input, const char delim)
{
  // Parse into array separated by delim
  // Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813
  std::vector<String> args = {};
  // String args[20];
  // int argCount = 0;
  while (input.length() > 0)
  {
    int index = input.indexOf(delim);
    if (index == -1)
    { // No space found
      args.push_back(input);
      break;
    }
    else
    {
      args.push_back(input.substring(0, index));
      input = input.substring(index + 1);
    }
  }

  return args;
}