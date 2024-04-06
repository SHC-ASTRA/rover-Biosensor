// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <utility/imumaths.h>
#include <FastLED.h>
// Our own resources
#include "AstraMotors.h"
#include "AstraCAN.h"
#include "AstraSensors.h"
#include "TeensyThreads.h"

using namespace std;


#define LED_STRIP_PIN     10
#define NUM_LEDS 38

int led_rbg[3] = {0, 300, 0}; //When using multiple colors, use 255 max, when doing R/B/G use 800-900 for best brightness
int led_counter = 0;

CRGB leds[NUM_LEDS];


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGNSS;

//Sensor declarations
Adafruit_BNO055 bno = Adafruit_BNO055();

//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(2, 1, false, 50, 1.00F);//Front Left
AstraMotors Motor2(4, 1, false, 50, 1.00F);//Back Left
AstraMotors Motor3(1, 1, true, 50, 1.00F);//Front Right
AstraMotors Motor4(3, 1, true, 50, 1.00F);//Back Right

AstraMotors motorList[4] = {Motor1, Motor2, Motor3, Motor4};//Left motors first, Right motors Second


//Prototypes
int findRotationDirection(float current_direction, float target_direction);
bool autoTurn(int time,float target_direction);
void turnCW();
void turnCCW();
void Stop();
void goForwards(float speed);
void goBackwards(float speed);
void loopHeartbeats();
void outputBno();
void outputBmp();
void outputGPS();
void setLED(int r_val, int b_val, int g_val);




unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;


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

    // Initalization for using CAN with the sparkmax
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    pinMode(20, INPUT_PULLUP); //Needed for IMU to work on PCB


    FastLED.addLeds<WS2812, LED_STRIP_PIN, GRB>(leds, NUM_LEDS);
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(led_rbg[0], led_rbg[1], led_rbg[2]);
      FastLED.show();
      delay(10);
    }




  //--------------------//
  // Initialize Sensors //
  //--------------------//
    if(!bno.begin())
    {
      Serial.println("!BNO failed to start...");
    }else{
      Serial.println("BNO055 Started Successfully");
    }if(!bmp.begin_I2C()) {
      Serial.println("bmp not working");
    }else{
      Serial.println("bmp is working");
    }if(!myGNSS.begin()){
      Serial.println("GPS not working");
    }else{
      Serial.println("GPS is working");
    }

  initializeBMP(bmp);



// Setup for GPS
myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println(F("getTimePulseParameters failed! not Freezing..."));
  }

  // Print the CFG TP5 version
  Serial.print(F("UBX_CFG_TP5 version: "));
  Serial.println(timePulseParameters.version);

  timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin
  //timePulseParameters.tpIdx = 1; // Or we could select the TIMEPULSE2 pin instead, if the module has one

  // We can configure the time pulse pin to produce a defined frequency or period
  // Here is how to set the frequency:

  // While the module is _locking_ to GNSS time, make it generate 2kHz
  timePulseParameters.freqPeriod = 2000; // Set the frequency/period to 2000Hz
  timePulseParameters.pulseLenRatio = 0x55555555; // Set the pulse ratio to 1/3 * 2^32 to produce 33:67 mark:space

  // When the module is _locked_ to GNSS time, make it generate 1kHz
  timePulseParameters.freqPeriodLock = 1000; // Set the frequency/period to 1000Hz
  timePulseParameters.pulseLenRatioLock = 0x80000000; // Set the pulse ratio to 1/2 * 2^32 to produce 50:50 mark:space

  timePulseParameters.flags.bits.active = 1; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
  timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
  timePulseParameters.flags.bits.isFreq = 1; // Tell the module that we want to set the frequency (not the period)
  timePulseParameters.flags.bits.isLength = 0; // Tell the module that pulseLenRatio is a ratio / duty cycle (* 2^-32) - not a length (in us)
  timePulseParameters.flags.bits.polarity = 1; // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)

  // Now set the time pulse parameters
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println(F("setTimePulseParameters failed!"));
  }
  else
  {
    Serial.println(F("Success!"));
  }


  //Start heartbeat thread
  //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
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
  // Every 50 milliseconds, update the speed for all motors
  if(millis()-lastAccel >= 50){
    lastAccel = millis();
    for(int i = 0; i < 4; i++)
    {
      motorList[i].UpdateForAcceleration();
    }

    if(motorList[0].getControlMode() == 1)//send the correct duty cycle to the motors
    {
        for(int i = 0; i < 4; i++)
        {
          sendDutyCycle(Can0, motorList[i].getID(), motorList[i].getDuty());
        }
        
    }else{
        //pass for RPM control mode
    }
 
  }



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

    if (token == "ctrl") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 2){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            motorList[i].setDuty(stof(token));
            motorList[i+1].setDuty(stof(token));
            
            scommand.erase(0, pos + delimiter.length());
          }

          
        }else{
          //pass if command if control command is same as previous
        }
    }else if (token == "speedMultiplier") {         // Is looking for a command that looks like "ctrl,x" where 0<x<1
      scommand.erase(0, pos + delimiter.length());
      token = scommand.substr(0, pos);
      pos = scommand.find(delimiter);
      //Motor1.setMotorMultiplier(stof(token));
    }else if (token == "auto") {  // Commands for autonomy
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());
          prevCommand = command;
          pos = scommand.find(delimiter);
          token = scommand.substr(0, pos);


          if(token == "turningTo"){ // auto,turningTo
            bool success = false;

            scommand.erase(0, pos + delimiter.length());
            pos = scommand.find(delimiter); 
            token = scommand.substr(0, pos);
            scommand.erase(0, pos + delimiter.length());
            pos = scommand.find(delimiter);
            token2 = scommand.substr(0, pos);

            success = autoTurn(stof(token),stoi(token2));
            if(success)
            {
              Serial.println("turningTo,success");
            }else{
              Serial.println("turningTo,fail");
            }
          }else if(token == "forwards"){  // auto,forwards
            scommand.erase(0, pos + delimiter.length());
            pos = scommand.find(delimiter); 
            token = scommand.substr(0, pos);

            goForwards(stof(token));
          }else if(token == "backwards"){ // auto,backwards
            scommand.erase(0, pos + delimiter.length());
            pos = scommand.find(delimiter); 
            token = scommand.substr(0, pos);

            goBackwards(stof(token));
          }else if(token == "stop"){  // auto,stop
            Stop();
          }
        }else{
          //pass if command if control command is same as previous
        }
    }else if (token == "data") {  // Send data out
        
          scommand.erase(0, pos + delimiter.length());
          prevCommand = command;
          pos = scommand.find(delimiter);
          token = scommand.substr(0, pos);


          if(token == "sendGPS"){ // data,sendGPS

            outputGPS();

          }else if(token == "sendIMU"){ // data,sendIMU

            outputBno();

          }else if(token == "sendBMP"){ // data,sendBMP

            outputBmp();

          }else if(token == "everything"){  // data,everything

            outputGPS();
            outputBno();
            outputBmp();

          }else if(token == "getOrientation"){  // data,getOrientation
            Serial.printf("orientation,%f\n", getBNOOrient(bno));
          }
        
    } else if (token == "led_set") {    //set LED strip color format: led_set,r,b,g
      
      for(int i = 0; i < 3; i++)
      {
        scommand.erase(0, pos + delimiter.length());
        token = scommand.substr(0, pos);
        pos = scommand.find(delimiter);
        led_rbg[i] = stoi(token);
      }
      
      setLED(led_rbg[0], led_rbg[1], led_rbg[2]);

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

// Prints the output of the BNO in one line
void outputBno()
{
  float bnoData2[7];
  pullBNOData(bno,bnoData2);
  printf("bno,%f,%f,%f,%f,%f,%f,%f\n",bnoData2[0],bnoData2[1],bnoData2[2],bnoData2[3],bnoData2[4],bnoData2[5],bnoData2[6]);
}

// Prints the output of the GPS in one line
void outputGPS()
{
  float gpsData[3];
  getPosition(myGNSS, gpsData);
  printf("gps,%f,%f,%f\n",gpsData[0],gpsData[1],gpsData[2]);
}

// Prints the output of the BMP in one line
void outputBmp()
{
  float bmpData[3];
  pullBMPData(bmp, bmpData);
  printf("bmp,%f,%f,%f\n",bmpData[0],bmpData[1],bmpData[2]);
}

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW(){
  sendDutyCycle(Can0, 2, 0.6);
  sendDutyCycle(Can0, 4, 0.6);
  sendDutyCycle(Can0, 1, 0.6);
  sendDutyCycle(Can0, 3, 0.6);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW(){
  sendDutyCycle(Can0, 2, -0.6);
  sendDutyCycle(Can0, 4, -0.6);
  sendDutyCycle(Can0, 1, -0.6);
  sendDutyCycle(Can0, 3, -0.6);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop(){
  sendDutyCycle(Can0, motorList[0].getID(), 0);
  sendDutyCycle(Can0, motorList[1].getID(), 0);
  sendDutyCycle(Can0, motorList[2].getID(), 0);
  sendDutyCycle(Can0, motorList[3].getID(), 0);
}

// Tells the rover to go forwards
// Does not bypass acceleration
// Autonomy
void goForwards(float speed){
  motorList[0].setDuty(speed);
  motorList[1].setDuty(speed);
  motorList[2].setDuty(speed);
  motorList[3].setDuty(speed);
}

// Tells the rover to go backwards
// Does not bypass acceleration
// Autonomy
void goBackwards(float speed){
  float temp = (-1) * speed;
  motorList[0].setDuty(temp);
  motorList[1].setDuty(temp);
  motorList[2].setDuty(temp);
  motorList[3].setDuty(temp);
}

// Specify where the rover should turn
// and how long it should take before
// the rover decides that it has failed
// to turn
bool autoTurn(int time, float target_direction){
  int startTime = millis(); 
  unsigned long expectedTime;
  expectedTime = time;
  
  float current_direction = getBNOOrient(bno);
  bool turningRight = findRotationDirection(current_direction, target_direction);

  Serial.printf("StartTime: %d, Expected: %d",(int)startTime, (int)expectedTime);


  while(millis() - startTime < expectedTime){
    current_direction = getBNOOrient(bno);
    if(!((current_direction < target_direction + 2) && (current_direction > target_direction - 2))){
      
      turningRight = findRotationDirection(current_direction, target_direction);

      Serial.print("Turning to: ");
      Serial.println(target_direction);
      Serial.print("Currently at: ");
      Serial.println(getBNOOrient(bno));

      if(turningRight){
        turnCW();
      }else{
        turnCCW();
      }
    }else{
      Stop();
      return true;
    }
  }
  Stop();
  return false;
}


// Finds out which direction the rover should turn
int findRotationDirection(float current_direction, float target_direction){
  int cw_dist = target_direction - current_direction + 360;
  cw_dist %= 360;
  int ccw_dist = current_direction - target_direction + 360; 
  ccw_dist %= 360;

  if(cw_dist <= ccw_dist)
  {
    return 1;//Rotate CW if distance is 180 or less
  }else{
    return 0;//Rotate CCW if distance is greater than 180
  }
} 


// Magic that makes the SparkMax work with CAN
void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
      sendHeartbeat(Can0, 1);
      threads.delay(3);
      sendHeartbeat(Can0, 2);
      threads.delay(3);
      sendHeartbeat(Can0, 3);
      threads.delay(3);
      sendHeartbeat(Can0, 4);
      threads.delay(3);
      threads.yield();
    }
}


void setLED(int r_val, int b_val, int g_val)
{
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(r_val, b_val, g_val);
      FastLED.show();
      delay(10);
    }
}