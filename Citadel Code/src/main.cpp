// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
//#include <utility/imumaths.h>

using namespace std;


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


//Prototypes
void activateCapSer(int num, int truFal);
void activatePump(int num, int truFal);
void activateFan(int num, int truFal);


unsigned long clockTimer = millis();


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);
    digitalWrite(LED_PIN, LOW);


  //--------------------//
  // Initialize Camera //
  //--------------------//

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
  //----------------------------------//
  // Runs something at a set interval //
  // Useful for testing               //
  //----------------------------------//

  if(0){
    if((millis()-clockTimer)>50){
      clockTimer = millis();

    }
  }


  //------------------//
  // Command Receiving //
  //------------------//
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
  // The giant CMD helps with finding this place
  //
  // Commands will be received as a comma separated value string
  // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
  // The program parses the string so that each piece of data can be used individually
  // For examples of parsing data you can use the link below
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Command is equal to a line in the serial
    command.trim();                                 // I don't know why this is here, but it is important
    string delimiter = ",";                         // The key that decides where the command should be split
    size_t pos = 0;                                 // Standard parse variable
    string token;                                   // The current piece of the string being used.
    string token2;                                  // A secondary piece of the string saved.
    string scommand = command.c_str();              // Converts the Arduino String into a C++ string since they are different things
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
    String prevCommand;

    if (token == "fan") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 1){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            
            activateFan(i,stoi(token));
        
            scommand.erase(0, pos + delimiter.length());
          }

          
        }
    }else if(token == "pump") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 1){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            
            activatePump(i,stoi(token));
        
            scommand.erase(0, pos + delimiter.length());
          }

          
        }
    }else if(token == "capServo") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 2; i+= 1){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            
            activateCapSer(i,stoi(token));
        
            scommand.erase(0, pos + delimiter.length());
          }

          
        }
    }else if (token == "mainServo") {         // Is looking for a command that looks like "ctrl,x" where 0<x<1
      scommand.erase(0, pos + delimiter.length());
      token = scommand.substr(0, pos);
      pos = scommand.find(delimiter);

      //activate servo with speed being the token

    } else if (token == "ping") {
      Serial.println("pong");
    } else if (token == "time") {
      Serial.println(millis());
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

void activateFan(int num, int truFal){
    if(num == 0){

    }
    if(num == 1){
        
    }
    if(num == 2){
        
    }
}

void activateCapSer(int num, int truFal){
    if(num == 0){

    }
    if(num == 1){
        
    }
    if(num == 2){
        
    }
}

void activatePump(int num, int truFal){
    if(num == 0){

    }
    if(num == 1){
        
    }
    if(num == 2){
        
    }
    if(num == 3){
        
    }
}

// Reminder of how to ask motors
// void turnCCW(){
//   sendDutyCycle(Can0, 2, -0.6);
//   sendDutyCycle(Can0, 4, -0.6);
//   sendDutyCycle(Can0, 1, -0.6);
//   sendDutyCycle(Can0, 3, -0.6);
// }

// Reminder of how to ask motors
// void goForwards(float speed){
//   motorList[0].setDuty(speed);
//   motorList[1].setDuty(speed);
//   motorList[2].setDuty(speed);
//   motorList[3].setDuty(speed);
// }

// Magic that makes the SparkMax work with CAN
// void loopHeartbeats(){
//     Can0.begin();
//     Can0.setBaudRate(1000000);
//     Can0.setMaxMB(16);
//     Can0.enableFIFO();
//     Can0.enableFIFOInterrupt();

//     while(1){
//       sendHeartbeat(Can0, 1);
//       threads.delay(3);
//       sendHeartbeat(Can0, 2);
//       threads.delay(3);
//       sendHeartbeat(Can0, 3);
//       threads.delay(3);
//       sendHeartbeat(Can0, 4);
//       threads.delay(3);
//       threads.yield();
//     }
// }