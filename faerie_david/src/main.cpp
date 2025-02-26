/**
 * @file main.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief description
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <SPI.h>    // Fixes compilation issue with Adafruit BusIO
#include <cmath>  // for abs()
#include <ESP32Servo.h>
#include "Adafruit_SHT31.h"  // adafruit/Adafruit SHT31 Library
#include <Adafruit_AS7341.h> // adafruit/Adafruit AS7341 Library

#include "AstraMisc.h"
#include "project/FAERIE.h"
#include "AstraVicCAN.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

#define REV_PWM_MIN 1000  // mcs  -- -1.0 duty   
#define REV_PWM_MAX 2000  // mcs  -- 1.0 duty


//---------------------//
//  Component classes  //
//---------------------//

// Now controls SCABBARD NEO-550 motor
Servo neo550;

// Faerie HUM/TEMP sensor (in SCABBARD)
Adafruit_SHT31 sht31;

// Faerie Wavelength sensor (in PIXIE)
Adafruit_AS7341 as7341;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

uint32_t lastCtrlCmd = 0;

uint32_t lastDataSend = 0;

// Shake mode variables

const uint32_t SHAKEINTERVAL = 250;  // Shake interval

const uint32_t SHAKEDURATION = 2500;  // How long to shake

const float SHAKEOPTIONS[5] = {0.1, 0.2, 0.3, 0.4, 0.5};

uint32_t shakeStart = 0;  // millis value when shaking started

uint32_t lastShake = 0;  // Last millis value of shake update

bool shakeMode = false;// Whether or not currently shaking

int shakeDir = 1;// Positive or negative to shake in open or close dir


//--------------//
//  Prototypes  //
//--------------//

String getSHTData();
String getAS7341Data();


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
void setup() {
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_LASERS, OUTPUT);
    digitalWrite(PIN_LASERS, HIGH);  // HIGH by default at Mason's request


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);


    //-----------//
    //  Sensors  //
    //-----------//

    if (!sht31.begin(0x44)) {  // HUM/Temp
        Serial.println("Couldn't find SHT31!");
        // while(1) delay(1);
    } else {
        Serial.println("SHT Initialized.");
    }


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    // SCABBARD NEO-550 motor
    neo550.attach(PIN_SPARKMAX_PWM, REV_PWM_MIN, REV_PWM_MAX);
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
void loop() {
    //----------//
    //  Timers  //
    //----------//
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    // Telemetry
    // if (millis() - lastDataSend >= 1000) {
    //     lastDataSend = millis();

    //     Serial.println(getSHTData());
    // }

    // SCABBARD Shake
    if (shakeMode && millis() - lastShake >= SHAKEINTERVAL) {
        lastShake = millis();

        unsigned ind = rand() % 5;  // 0-4 inclusive, seeded by command

        if (ind < 0 || ind > 4)
            ind = 0;
        neo550.write(0);

        // Don't shake for longer than SHAKEDURATION
        if (shakeStart + SHAKEDURATION <= millis()) {
            shakeMode = false;
            neo550.write(0);
        }
    }

    // Motor timeout
    if (millis() - lastCtrlCmd >= 2000) {
        lastCtrlCmd = millis();
        neo550.writeMicroseconds((REV_PWM_MIN + REV_PWM_MAX) / 2);
        digitalWrite(PIN_LASERS, LOW);
    }


    //-------------//
    //  CAN Input  //
    //-------------//


    //------------------//
    //  UART/USB Input  //
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
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args);        // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable

        //--------//
        //  Misc  //
        //--------//
        /**/ if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle") {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        else if (command == "data") {

            /**/ if (args[1] == "sendtemp") {
                float temp = sht31.readTemperature();
                if (!isnan(temp)) {
                    Serial.println(temp);
                } else {
                    Serial.println("Failed to read temperature.");
                }
            }

            else if (args[1] == "sendhum") {
                float hum = sht31.readHumidity();
                if (!isnan(hum)) {
                    Serial.println(hum);
                } else {
                    Serial.println("Failed to read humidity.");
                }
            }

            else if (args[1] == "sendsht") {
                Serial.println(getSHTData());
            }
        }

        //----------//
        //  Motors  //
        //----------//

        else if (command == "ctrl") {
            lastCtrlCmd = millis();

            /**/ if (args[1] == "scabbard") {
                neo550.writeMicroseconds(map_d(args[2].toFloat(), -1.0, 1.0, REV_PWM_MIN, REV_PWM_MAX));
            }

            else if (args[1] == "shake") {
                lastCtrlCmd = millis();
                shakeMode = true;

                if (args[2] == "close")
                    shakeDir = 1;
                else if (args[2] == "open")
                    shakeDir = -1;
                else
                    shakeDir = 1;

                // Shake immediately
                lastShake = 0;
                shakeStart = millis();

                // Seed rand() for random duty cycles
                srand(millis());
            }
        }

        else if (command == "stop") {
            lastCtrlCmd = millis();

            shakeMode = false;
            neo550.write(0);
            digitalWrite(PIN_LASERS, LOW);
        }

        else if (command == "laser") {
            if (args[1] == "on")
                digitalWrite(PIN_LASERS, HIGH);
            else
                digitalWrite(PIN_LASERS, LOW);
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

// Poll SHT and format temperature and humidity data into String
// in format "faeriesht,`{temperature}`,`{humidity}`"
// Temperature and humidity are rounded to 2 decimal places
String getSHTData(void) {
    float temp, hum;
    sht31.readBoth(&temp, &hum);

    String res;       // concatenated result of temp and hum data
    res.reserve(23);  // Memory safety. Ex: "faeriesht,150.2,138.6"
    res += "faeriesht,";

    // Verify temperature reading
    if (!isnan(temp))
        res += String(temp, 1);  // Ex. 25.38 (Use constructor to round 1 decimal place)
    else
        res += "999.9";

    res += ',';

    // Verify humidity reading
    if (!isnan(hum))
        res += String(hum, 1);  // Ex. 28.25
    else
        res += "999.9";

    return res;
}

String getAS7341Data(void) {
    if (!as7341.readAllChannels()) {
        Serial.println("Error reading all channels!");
        return;
    }
    
    // Print out the stored values for each channel
    Serial.print("F1 415nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_415nm_F1));
    Serial.print("F2 445nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_445nm_F2));
    Serial.print("F3 480nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_480nm_F3));
    Serial.print("F4 515nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_515nm_F4));
    Serial.print("F5 555nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_555nm_F5));
    Serial.print("F6 590nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_590nm_F6));
    Serial.print("F7 630nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_630nm_F7));
    Serial.print("F8 680nm : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_680nm_F8));

    Serial.print("Clear    : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_CLEAR));

    Serial.print("Near IR  : ");
    Serial.println(as7341.getChannel(AS7341_CHANNEL_NIR));

    Serial.println("");
    
}
