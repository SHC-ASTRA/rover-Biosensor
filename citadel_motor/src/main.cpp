/**
 * @file main.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Citadel's secondary microcontroller; controls 4 stepper motors
 *
 */

//------------//
//  Includes  //
//------------//

#include "AstraMisc.h"
#include "project/CITADEL.h"
#include <AccelStepper.h>

//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

#define COMMS_UART Serial // Serial1 to talk to main mcu
#define dirPin1 14
#define stepPin1 32
#define dirPin2 15
#define stepPin2 33
#define dirPin3 12
#define stepPin3 27
#define dirPin4 20
#define stepPin4 22
#define motorInterfaceType 1

//---------------------//
//  Component classes  //
//---------------------//

AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);
AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
hw_timer_t *Timer0_Cfg = NULL;
bool pump1On = 0, pump2On = 0, pump3On = 0, pump4On = 0;
int pos1, pos2, pos3, pos4;

//----------//
//  Timing  //
//----------//
void IRAM_ATTR Timer0_ISR()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');

        input.trim();                  // Remove preceding and trailing whitespace
        std::vector<String> args = {}; // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');  // Separate `input` by commas and place into args vector
        args[0].toLowerCase();         // Make command case-insensitive
        String command = args[0];      // To make processing code more readable

        //--------//
        //  Misc  //
        //--------//
        /**/
        if (args[0] == "ping")
            Serial.println("pong");
        else if (args[0] == "pump")
        {
            switch (args[1].toInt())
            {
            case 1:
                pump1On = 1;
                pos1 = args[2].toInt();
                break;
            case 2:
                pump2On = 1;
                pos2 = args[2].toInt();
                break;
            case 3:
                pump3On = 1;
                pos3 = args[2].toInt();
                break;
            case 4:
                pump4On = 1;
                pos4 = args[2].toInt();
                break;
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//
    }
}

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

    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.print("Ready");
    // stepper.setMaxSpeed(1000);
    pinMode(5, OUTPUT);
    pinMode(19, OUTPUT);
    digitalWrite(5, LOW);
    digitalWrite(19, LOW);
    stepper1.setMaxSpeed(1000);
    stepper2.setMaxSpeed(1000);
    stepper3.setMaxSpeed(1000);
    stepper4.setMaxSpeed(1000);
    stepper1.setAcceleration(200);
    // stepper2.setAcceleration(200);
    // stepper3.setAcceleration(200);
    // stepper4.setAcceleration(200);
    // stepper1.setCurrentPosition(0);
    // stepper2.setCurrentPosition(0);
    // stepper3.setCurrentPosition(0);
    // stepper4.setCurrentPosition(0);
    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);
    timerAlarmEnable(Timer0_Cfg);

    //-----------//
    //  Sensors  //
    //-----------//

    //--------------------//
    //  Misc. Components  //
    //--------------------//
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

    // if (Serial.available())
    // {
    //     String input = Serial.readStringUntil('\n');

    //     input.trim();                  // Remove preceding and trailing whitespace
    //     std::vector<String> args = {}; // Initialize empty vector to hold separated arguments
    //     parseInput(input, args, ',');  // Separate `input` by commas and place into args vector
    //     args[0].toLowerCase();         // Make command case-insensitive
    //     String command = args[0];      // To make processing code more readable

    //     //--------//
    //     //  Misc  //
    //     //--------//
    //     /**/

    //     //-----------//
    //     //  Sensors  //
    //     //-----------//

    //     //----------//
    //     //  Motors  //
    //     //----------//
    // }

    if(pump1On)
    {
        stepper1.moveTo(pos1);
        while(stepper1.currentPosition() != pos1)
        {
            stepper1.run();
        }
        stepper1.moveTo(0);
        while(stepper1.currentPosition() != 0)
        {
            stepper1.run();
        }
        pump1On=0;
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
