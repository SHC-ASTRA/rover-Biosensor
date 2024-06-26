/*
 * ASTRA Biosensor
 * FAERIE Embedded Code
 */

//----------//
// Includes //
//----------//

// Standard Includes
#include <Arduino.h>

#include <cmath>  // for abs()
#include <vector>

// Our own resources
#include "FAERIE.h"


//-----------//
// Constants //
//-----------//

unsigned CAN_ID = 6;

//------------------------//
// Classes for components //
//------------------------//

// Basic 5V PWM servo for SCABBARD
Servo servo;


// SHT 31 in SCABBARD
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // Faerie HUM/TEMP Sensor

// Last millis value that sht temp and hum data was sent to socket
uint32_t lastDataSend = 0;


// Setting up for CAN0 line
AstraFCAN Can0;

// AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(CAN_ID, 1, false, 50, 0.50F);  // Drill

// Last millis value that the motor was sent a duty cycle
unsigned long lastAccel;


// Shake mode variables

// Shake interval
const uint32_t SHAKEINTERVAL = 250;

// How long to shake
const uint32_t SHAKEDURATION = 2500;

const float SHAKEOPTIONS[5] = {0.1, 0.2, 0.3, 0.4, 0.5};

// millis value when shaking started
uint32_t shakeStart = 0;

// Last millis value of shake update
uint32_t lastShake = 0;

// Whether or not currently shaking
bool shakeMode = false;

// Positive or negative to shake in open or close dir
int shakeDir = 1;


// How long to wait before stopping the motor after the last cmd was sent
const uint32_t MOTORTIMEOUT = 30'000;

// Last millis value that a motor command was received
// Used to stop the motor after MOTORTIMEOUT
uint32_t lastMotorCmd = 0;



//------------//
// Prototypes //
//------------//

void loopHeartbeats();
String getSHTData(void);


//-------------//
// Begin Setup //
//-------------//

void setup() {
    //-----------------//
    // Initialize Pins //
    //-----------------//

    Serial.begin(SERIAL_BAUD);
    COMMS_UART.begin(COMMS_UART_BAUD);  // for comms with Arm Socket Teensy/RasPi

    // Teensy built-in LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Faerie drill lasers
    pinMode(PIN_LASER_NMOS, OUTPUT);
    digitalWrite(PIN_LASER_NMOS, LOW);

    // SCABBARD Servo
    servo.attach(PIN_SERVO_PWM, SERVO_MIN, SERVO_MAX);

    // LED stays on for 2 seconds to show powered on
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // ------- //
    //   CAN   //
    // ------- //

    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    //--------------------//
    // Initialize Sensors //
    //--------------------//

    if (!sht31.begin(0x44)) {  // HUM/Temp
        Serial.println("Couldn't find SHT31!");
        // while(1) delay(1);
    } else {
        Serial.println("SHT Initialized.");
    }


    //-----------//
    // Heartbeat //
    //-----------//

    // Heartbeat propogation
    threads.addThread(loopHeartbeats);
}



//------------//
// Begin Loop //
//------------//
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
    // Accelerate the motors
    if (millis() - lastAccel >= 50) {
        lastAccel = millis();
        Motor1.UpdateForAcceleration();

        if (Motor1.getControlMode() == 1)  // send the correct duty cycle to the motors
        {
            sendDutyCycle(Can0, CAN_ID, Motor1.getDuty());

        } else {
            // pass for RPM control mode
        }
    }



    // Send SHT temp and hum data to socket once per second
    if (millis() - lastDataSend >= 1000) {
        lastDataSend = millis();

        COMMS_UART.println(getSHTData());
    }



    // Shake
    if (shakeMode && millis() - lastShake >= SHAKEINTERVAL) {
        lastShake = millis();

        unsigned ind = rand() % 5;  // 0-4 inclusive, seeded by command

        if (ind < 0 || ind > 4)
            ind = 0;

        Motor1.setDuty(shakeDir * SHAKEOPTIONS[ind]);

        // Don't shake for longer than SHAKEDURATION
        if (shakeStart + SHAKEDURATION <= millis()) {
            shakeMode = false;
            Motor1.setDuty(0);
        }
    }



    // Motor timeout
    if (millis() - lastMotorCmd >= MOTORTIMEOUT) {
        Motor1.setDuty(0);
        shakeMode = false;
    }



    //-------------------//
    // Command Receiving //
    //-------------------//
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
    //
    // Commands will be received as a comma separated value string
    // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"

    // Allows to use the same code to parse input wherever it comes from
    bool inputAvailable = false;
    int inputMethod;
    if (Serial.available()) {
        inputAvailable = true;
        inputMethod = 0;
    } else if (COMMS_UART.available()) {
        inputAvailable = true;
        inputMethod = COMMS_UART_NUM;
    }

    if (inputAvailable) {
        // Output to be sent to either Serial or COMMS_UART depending on which was used
        String output = "";

        String input;
        if (inputMethod == 0)                      // Input comes via USB
            input = Serial.readStringUntil('\n');  // Take str input from Serial

        else if (inputMethod == COMMS_UART_NUM)        // Input comes via UART from Arm
            input = COMMS_UART.readStringUntil('\n');  // Take str input from UART


        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');   // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable


        // Comes from ROS -> socket raspi ->UART-> socket teensy 4.1 ->UART-> FAERIE
        if (command == "faerie") {
            // Remove first argument, which is "faerie" to tell socket teensy to redirect to faerie
            args.erase(args.begin());
            // Our command is not "faerie", but what comes after it
            command = args[0].toLowerCase();
        }

        //--------------------//
        // Command Processing //
        //--------------------//

        //
        // Commands have been documented here:
        // https://docs.google.com/spreadsheets/d/16RYq-beKFbWoqob2tEWtEd6SiWK38EuTyW-QRfZV-Ow/edit#gid=532390044
        //

        //---------//
        // general //
        //---------//
        /**/ if (command == "ping") {
            output += "pong\n";
        }

        else if (command == "time") {
            output += millis();
            output += '\n';
        }

        else if (command == "line") {
            output += "-------------\n";
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else
                digitalWrite(LED_BUILTIN, LOW);
        }

        else if (command == "stop") {
            Motor1.setDuty(0);
            shakeMode = false;
        }

        //------//
        // ctrl //
        //------//
        else if (command == "ctrl") {  // ctrl //
            String subcommand = args[1].toLowerCase();


            /**/ if (subcommand == "duty") {
                // CW/+ = CLOSE, CCW/- = OPEN
                lastMotorCmd = millis();

                float val = args[2].toFloat();

                // Make it easier to stop the motor with the slider
                if (abs(val) < 0.03)
                    val = 0;

                // Safety check
                if (isnan(val) || abs(val) > 1) {
                    val = 0;
                    output += "faerieerror,invalidduty";
                }

                Motor1.setDuty(val);

                // Stop shake if duty is 0
                if (val == 0)
                    shakeMode = false;
            }

            else if (subcommand == "shake") {
                shakeMode = true;
                lastMotorCmd = millis();

                if (args[2] == "close")
                    shakeDir = 1;
                if (args[2] == "open")
                    shakeDir = -1;
                else
                    shakeDir = 1;

                // Shake immediately
                lastShake = 0;
                shakeStart = millis();

                // Seed rand() for random duty cycles
                srand(millis());
            }

            else if (subcommand == "servo") {
                servo.write(args[2].toInt());
            }

            else if (subcommand == "stop") {
                Motor1.setDuty(0);
                shakeMode = false;
            }

            else if (subcommand == "id") {
                identifyDevice(Can0, CAN_ID);
            }

            else if (subcommand == "shtheater") {
                if (args[2] == "on")
                    sht31.heater(true);
                else
                    sht31.heater(false);
            }

            else if (subcommand == "laser") {
                if (args[2] == "on")
                    digitalWrite(PIN_LASER_NMOS, HIGH);
                else
                    digitalWrite(PIN_LASER_NMOS, LOW);
            }

            else if (subcommand == "loadsht") {
                sht31.begin(0x44);
            }

        }

        //------//
        // data //
        //------//
        else if (command == "data") {  // data //
            String subcommand = args[1].toLowerCase();


            /**/ if (subcommand == "sendtemp") {
                float temp = sht31.readTemperature();
                if (!isnan(temp)) {
                    output += temp;
                    output += '\n';
                } else {
                    output += "Failed to read temperature.\n";
                }
            }

            else if (subcommand == "sendhum") {
                float hum = sht31.readHumidity();
                if (!isnan(hum)) {
                    output += hum;
                    output += '\n';
                } else {
                    output += "Failed to read humidity.\n";
                }
            }

            else if (subcommand == "sendheater") {
                if (sht31.isHeaterEnabled()) {
                    output += "true\n";
                } else {
                    output += "false\n";
                }
            }

            else if (subcommand == "sendsht") {
                output += getSHTData();
                output += '\n';
            }
        }

        //--------------//
        // END COMMANDS //
        //--------------//

        if (output.length() > 1) {
            if (inputMethod == 0)
                Serial.print(output);
            else if (inputMethod == COMMS_UART_NUM)
                COMMS_UART.print(output);
        }
    }
}



//-------------------------------------------------------//
//                                                       //
//    ///////////    //\\          //      //////////    //
//    //             //  \\        //    //              //
//    //             //    \\      //    //              //
//    //////         //      \\    //    //              //
//    //             //        \\  //    //              //
//    //             //          \\//    //              //
//    //             //           \//      //////////    //
//                                                       //
//-------------------------------------------------------//


// clang-format off: to better sync with other people's code
void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
        sendHeartbeat(Can0, CAN_ID);
        threads.delay(12);
        threads.yield();
    }

}
// clang-format on

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
