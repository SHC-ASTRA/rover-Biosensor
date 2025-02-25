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

#define COMMS_UART Serial1 // Serial1 to talk to main mcu


//---------------------//
//  Component classes  //
//---------------------//

AccelStepper stepper1 = AccelStepper(1, PIN_STEP_1_STEP, PIN_STEP_1_DIR);
AccelStepper stepper2 = AccelStepper(1, PIN_STEP_2_STEP, PIN_STEP_2_DIR);
AccelStepper stepper3 = AccelStepper(1, PIN_STEP_3_STEP, PIN_STEP_3_DIR);
AccelStepper stepper4 = AccelStepper(1, PIN_STEP_4_STEP, PIN_STEP_4_DIR);

hw_timer_t *Timer0_Cfg = NULL;
bool pump1On = 0, pump2On = 0, pump3On = 0, pump4On = 0;
int pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

// void IRAM_ATTR Timer0_ISR()
// {
    
// }


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

    pinMode(PIN_M0, OUTPUT);
    pinMode(PIN_M1, OUTPUT);
    digitalWrite(PIN_M0, LOW);
    digitalWrite(PIN_M1, LOW);

    //------------------//
    //  Communications  //
    //------------------//

    COMMS_UART.begin(COMMS_UART_BAUD);
    COMMS_UART.print("Ready");

    //-----------//
    //  Sensors  //
    //-----------//

    //--------------------//
    //  Misc. Components  //
    //--------------------//

    // stepper.setMaxSpeed(1000);
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

    // Timer0_Cfg = timerBegin(0, 80, true);
    // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    // timerAlarmWrite(Timer0_Cfg, 1000, true);
    // timerAlarmEnable(Timer0_Cfg);
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
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    if (pump1On)
    {
        stepper1.moveTo(pos1);
        while (stepper1.currentPosition() != pos1)
        {
            stepper1.run();
        }
        stepper1.moveTo(0);
        while (stepper1.currentPosition() != 0)
        {
            stepper1.run();
        }
        pump1On = 0;
    }

    //-------------//
    //  CAN Input  //
    //-------------//

    // No CAN input for Citadel motor mcu


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

    if (COMMS_UART.available())
    {
        String input = COMMS_UART.readStringUntil('\n');

        input.trim();                  // Remove preceding and trailing whitespace
        std::vector<String> args = {}; // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');  // Separate `input` by commas and place into args vector
        args[0].toLowerCase();         // Make command case-insensitive
        String command = args[0];      // To make processing code more readable

        //--------//
        //  Misc  //
        //--------//
        /**/ if (args[0] == "ping") {
            COMMS_UART.println("pong");
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//

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
