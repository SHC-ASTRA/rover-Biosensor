/**
 * @file main.cpp
 * @author Karl (you@domain.com)
 * @author David (you@domain.com)
 * @brief Controls Citadel's fans, servos, LSS, vibration motor, and relays commands for steppers
 *
 */


//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <cmath>
#include <LSS.h>


#include <ESP32Servo.h>
#include <AccelStepper.h>

#include "AstraMisc.h"
#include "project/CITADEL.h"
#include "AstraVicCAN.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

#define LSS_ID 254

//---------------------//
//  Component classes  //
//---------------------//

SoftwareSerial Serial_LSS(PIN_LYNX_RX, PIN_LYNX_TX);

SoftwareSerial Serial_Pi(PIN_PI_RX, PIN_PI_TX);

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
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);

    // Fans
    pinMode(PIN_FAN_1, OUTPUT);
    pinMode(PIN_FAN_2, OUTPUT);
    pinMode(PIN_FAN_3, OUTPUT);
    digitalWrite(PIN_FAN_1, LOW);
    digitalWrite(PIN_FAN_2, LOW);
    digitalWrite(PIN_FAN_3, LOW);

    // Vibrator
    pinMode(PIN_VIBMOTOR, OUTPUT);
    digitalWrite(PIN_VIBMOTOR, LOW);


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);

    ESP32Can.begin(TWAI_SPEED_1000KBPS, PIN_CTX, PIN_CRX);


    //-----------//
    //  Sensors  //
    //-----------//

    
    //--------------------//
    //  Misc. Components  //
    //--------------------//

    servo1.attach(PIN_PWMSERVO_1);
    servo2.attach(PIN_PWMSERVO_2);
    servo3.attach(PIN_PWMSERVO_3);

    LSS::initBus(Serial_LSS, LSS_BAUD);
    myLSS.setAngularStiffness(0);
    myLSS.setAngularHoldingStiffness(0);
    myLSS.setAngularAcceleration(15);
    myLSS.setAngularDeceleration(15);
    Serial.println("Smart servo has started");

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

    if (fansOn && (millis() - prevFanTime >= fansTimer))
    {
        digitalWrite(PIN_FAN_1, LOW);
        digitalWrite(PIN_FAN_2, LOW);
        digitalWrite(PIN_FAN_3, LOW);
        fansOn = 0;
        fansTimer = 0;
    }
    else if (fanOn_1 && (millis() - prevFanTime_1 >= fanTimer_1))
    {
        digitalWrite(PIN_FAN_1, LOW);
        fanOn_1 = 0;
        fanTimer_1 = 0;
    }
    else if (fanOn_2 && (millis() - prevFanTime_2 >= fanTimer_2))
    {
        digitalWrite(PIN_FAN_2, LOW);
        fanOn_2 = 0;
        fanTimer_2 = 0;
    }
    else if (fanOn_3 && (millis() - prevFanTime_3 >= fanTimer_3))
    {
        digitalWrite(PIN_FAN_3, LOW);
        fanOn_3 = 0;
        fanTimer_3 = 0;
    }

    //-------------//
    //  CAN input  //
    //-------------//

    if (vicCAN.readCan())
    {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        Serial.print("VicCAN: ");
        Serial.print(commandID);
        Serial.print("; ");
        if (canData.size() > 0)
        {
            for (const double &data : canData)
            {
                Serial.print(data);
                Serial.print(", ");
            }
        }
        Serial.println();

        /**/ if (commandID == CMD_PING)
        {
            vicCAN.respond(1); // "pong"
            Serial.println("Received ping over CAN");
        }

        else if (commandID == CMD_LSS_TURNBY_DEG)
        {

            if (canData.size() == 1)
            {
                if (canData[0] == -1) {
                    myLSS.wheel(-20);
                }
                else if (canData[0] == 0) {
                    myLSS.wheel(0);
                }
                else if (canData[0] == 1) {
                    myLSS.wheel(20);
                }
            }
        }

        else if (commandID == CMD_PWMSERVO_SET_DEG)
        {
            if (canData.size() == 2)
            {
                switch (static_cast<int>(canData[0]))
                {
                case 1:
                    servo1.write(canData[1]);
                    break;
                case 2:
                    servo2.write(canData[1]);
                    break;
                case 3:
                    servo3.write(canData[1]);
                    break;
                default:
                    break;
                }
            }
        }

        else if (commandID == CMD_DCMOTOR_CTRL)
        {
            if (canData.size() == 1)
            {
                if (canData[0] == 1)
                    digitalWrite(PIN_VIBMOTOR, HIGH);
                else if (canData[0] == 0)
                    digitalWrite(PIN_VIBMOTOR, LOW);
            }
        }

        else if (commandID == CMD_STEPPER_CTRL)
        {
            if (canData.size() == 2) {
                MOTOR_UART.printf("pump,%d,%d\n", canData[1], canData[2]);
            }
        }

        else if (commandID == CMD_CITADEL_FAN_CTRL) {
            if (canData.size() == 2) {
                switch (static_cast<int>(canData[0])) {
                case 1:
                    digitalWrite(PIN_FAN_1, HIGH);
                    fanOn_1 = 1;
                    fanTimer_1 = canData[1];
                    prevFanTime_1 = millis();
                    break;
                case 2:
                    digitalWrite(PIN_FAN_2, HIGH);
                    fanOn_2 = 1;
                    fanTimer_2 = canData[1];
                    prevFanTime_2 = millis();
                    break;
                case 3:
                    digitalWrite(PIN_FAN_3, HIGH);
                    fanOn_3 = 1;
                    fanTimer_3 = canData[1];
                    prevFanTime_3 = millis();
                    break;
                default:
                    break;
                }
            }
        }
    }

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
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        Serial.println(input);

        input.trim();                    // Remove preceding and trailing whitespace
        std::vector<String> args = {}; // Initialize empty vector to hold separated arguments
        parseInput(input, args); // Separate `input` by commas and place into args vector
        args[0].toLowerCase();         // Make command case-insensitive
        String command = args[0];      // To make processing code more readable

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

        else if (command == "led")
        {
            digitalWrite(LED_BUILTIN, !ledState);
            ledState = !ledState;
        }

        else if (args[0] == "can_relay_tovic")
        {
            vicCAN.relayFromSerial(args);
        }

        else if (args[0] == "can_relay_mode")
        {
            if (args[1] == "on")
            {
                vicCAN.relayOn();
            }
            else if (args[1] == "off")
            {
                vicCAN.relayOff();
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//

        // Fan
        if (args[0] == "fans")  // Is looking for a command that looks like "fans,1,time"
        {
            Serial.println(args[0]);
            digitalWrite(PIN_FAN_1, args[1].toInt());
            digitalWrite(PIN_FAN_2, args[1].toInt());
            digitalWrite(PIN_FAN_3, args[1].toInt());
            fansOn = 1;
            fansTimer = args[2].toInt();
            prevFanTime = millis();
            Serial.println("Fans Activated");
        }

        else if (args[0] == "fan")  // Is looking for a command that looks like "fan,2,1,time"
        {
            Serial.println("Reached fan");
            switch (args[1].toInt())
            {
            case 1:
                digitalWrite(PIN_FAN_1, args[2].toInt());
                fanOn_1 = 1;
                fanTimer_1 = args[3].toInt();
                prevFanTime_1 = millis();
                break;
            case 2:
                digitalWrite(PIN_FAN_2, args[2].toInt());
                fanOn_2 = 1;
                fanTimer_2 = args[3].toInt();
                prevFanTime_2 = millis();
                break;
            case 3:
                digitalWrite(PIN_FAN_3, args[2].toInt());
                fanOn_3 = 1;
                fanTimer_3 = args[3].toInt();
                prevFanTime_3 = millis();
                break;
            default:
                break;
            }
            Serial.println("Fan Activated");
        }
        
        else if (args[0] == "servo")
        {
            switch (args[1].toInt())
            {
            case 1:
                servo1.write(args[2].toInt());
                break;
            case 2:
                servo2.write(args[2].toInt());
                break;
            case 3:
                servo3.write(args[2].toInt());
                break;
            default:
                break;
            }
        }

        else if (args[0] == "shutdown")
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

        // else if (args[0] == "pump")
        // {
        //     MOTOR_UART.print(input);
        // }

        // else if (args[0] == "Smartservo")
        // {
        //   if (command != input)
        //   {
        //     if (args[1] == "Relative")
        //     {
        //       myLSS.moveRelative(((args[2]).toInt()) * 10);
        //       Serial.println(myLSS.getPosition());
        //     }
        //     else if (args[1] == "FullRetract")
        //     {
        //       myLSS.moveRelative(((args[2]).toInt()) * 10);
        //       Serial.println(myLSS.getPosition());
        //     }
        //     else if (args[1] == "FullRetract")
        //     {
        //       myLSS.move(-900);
        //       Serial.println("Full Retractig CITADEL arm");
        //     }
        //     else if (args[1] == "Half")
        //     {
        //       myLSS.move(0);
        //       Serial.println("Setting arm to half extend");
        //     }
        //     else if (args[1] == "Extend")
        //     { //
        //       myLSS.moveRelative(20);
        //       Serial.println("Extending CITADEL arm");
        //     }
        //     else if (args[1] == "Retract")
        //     { //
        //       myLSS.moveRelative(-20);
        //       Serial.println("Retractig CITADEL arm");
        //     }
        //     else if (args[1] == "Reset")
        //     {
        //       myLSS.reset();
        //       Serial.println("Servo Reset");
        //     }
        //   }
        // }
    }

    // if (MOTOR_UART.available()) {
    //     String input = MOTOR_UART.readStringUntil('\n');
    //     input.trim();
    //     COMMS_UART.println(input);
    // }
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
