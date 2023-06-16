/**
 * 
 * @file main.cpp
 * @author Loïc Reboursière - UMONS, CLick (loic.reboursiere@umons.ac.be)
 * @brief This code is involved in the development of a 3D motorized LiDAR scanner based on servomotors. It controls the two servomotors (horizontal and vertical) from Adafruit pantilt which moves a LiDAR sensor.
 * Two scans can be performed and launched by the use of a button. The first scan is meant to be the background scene and the second scan the scene with a specific object.
 * After the second scan, the two clouds are subtracted to obtain the cloud points relative to the added object.
 * This cloud point is meant to be used to extract gravity center and dimension of the object.
 * 3DLidarScanner-v1 – CLICK - UMONS (Loïc Reboursière) is free software: you can redistribute it and/or modify it under the terms of the Apache License Version 2.0. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Apache License Version 2.0 for more details.
 * You should have received a copy of the Apache License Version 2.0 along with this program.  If not, see http://www.apache.org/licenses/
 * Each use of this software must be attributed to University of MONS – CLICK (Loïc Reboursière).
 * Any other additional authorizations may be asked to avre@umons.ac.be. 
 * @version 0.1
 * @date 2023-05-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */



//-----------------------------------------------
// 3D LiDAR scanner based on ESP32-C3-DevKitM1
//------------------------------------------------
// Elements :
// * TFMini-S
// * Adafruit pantilt with 2 Tower Pro sg90 servo motors
// * ESP32-C3-Devkit-M1
// * External power plug (5V, 2A) with positive center
// Lib : 
// * Fork of ESP32Servo : https://github.com/loicreboursiere/ESP32Servo || LGPL  
// * TFMini-Plus library (compatible TFMini-S): https://github.com/budryerson/TFMini-Plus || None
// * https://github.com/me-no-dev/ESPAsyncWebServer || LGPL
//    * Modifications
//      * in AsyncWebSocket.cpp, replace return IPAddress(0U) by IPAddress((uint32_t)0) 
//          >> SRC : https://github.com/me-no-dev/ESPAsyncWebServer/issues/1101
//      * in WebAuthentification.cpp, replace mbedtls_md5_starts(&_ctx) by mbedtls_md5_starts_ret(&_ctx)
//            >> SRC : https://github.com/me-no-dev/ESPAsyncWebServer/issues/1147
//
//
// Connections : 
// * TFMini-Plus RX (white) >> 5
// * TFMini-Plus RX (green) >> 4
// * TFMini-Plus + >> + external Power Plug
// * TFMini-Plus GND >> GND ESP32-C3-Devkit-M1
// * horizServoPin >> 19
// * vertServoPin >> 18
// * horizServo + >> + external power plug
// * horizServo GND >> GND external power plug
// * vertServo + >> + external power plug
// * vertServo GND >> GND external power plug
// * GND ESP32-C3-Devkit-M1 >> GND external power plug
// Sources : 
// * https://how2electronics.com/how-to-use-tfmini-s-lidar-distance-sensor-with-arduino/
// * https://github.com/TravisLedo/LidarScanRender
// Author :
// * loic.reboursiere@umons.ac.be  

// TODO : 
// 16/12/2022 : Check if SIGNAL WEAK LiDAR message means distance = 0 

#include <Arduino.h>

#ifdef ESP32
#include <ESP32Servo.h>
//#include <SPIFFS.h>
//#include "ESPAsyncWebServer.h"
//const char *renderer = 
//#include "Renderer.h"
#else
#include <Servo.h>
#endif

#include <TFMPlus.h>
//#include <WiFi.h>
//s#include "stdio.h"
//#include "time.h"

#include "variables.h"
#include "computations.h"
#include "scanner.h"
#include "scannerServer.h"

//#define TFMINI_BAUDRATE 115200
//#define TF_SERIAL Serial1
//#define TF_RX 5
//#define TF_TX 4
#define CMD_BTN_PIN 2

#define OUTPUT_JS 0
#define OUTPUT_SERIAL 0

//USed to output prints functions to serial Monitor
#define DEBUG 1
#define VERBOSE 1

String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

Servo myservo_horiz; 
Servo myservo_vert;

//TFMini tfmini;
TFMPlus tfmS;// The used sensor is the TFMini-S compatible with the TFMiniPlus

int pointIdx = -1;
//HardwareSerial * TFSerComm;
String ledState;
// PINS
int led_test = 13;

#ifdef ESP32
int horizServoPin = 19;
int vertServoPin = 18;//18;
#define TIMER_WIDTH 10
#else
int horizServoPin = 8;
int vertServoPin = 9;
#endif

//float pi = 3.14159265;
//float deg2rad = pi / 180.0;

#define MAX_PROG_STEP 2 // Max nb of steps the program can go through
int nbBtnClick = 0; //Count click on the button : 1 is scan background, 2 is scan with object, 3 is send center information
int nbBtnClick_previous = 0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;  
int buttonState;  
int lastButtonState = LOW;

char backgroundSavePath [] = "/";
char withObjectSavePath [] = "/";

// Command from webpage
const char * PARAM_INPUT_1 = "prog_step";

// To be able to get time to write in cloudpoint file
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;


///// Matrix testing
//const int rows = 3;
//const int cols = 3;
//int bg[ rows ][ cols ]  = { { 10, 10, 10 }, { 10, 10, 10 }, { 10, 10, 10} };
//int currentImg[ rows ][ cols ] = { { 4, 5, 6 }, { 1, 2, 3 }, { 3, 7, 5} };
//int result[ rows ][ cols ];
const uint8_t rows = uint16_t( (v_stop_angle - v_start_angle) / vertServoStep );
const uint8_t cols = uint16_t( (h_stop_angle - h_start_angle) / horizServoStep );

// TODO : check how to convert rows and cols in const 
// rows = 50 & cols = 150
//uint8_t * bg = new uint8_t [ 50 ];// = new uint8_t[ rows * cols ];
//uint8_t bg [ rows ][ cols];

// CloudPoint * cloud           = new CloudPoint [ 1500 ]
// CloudPoint cloud            [ ARRAY_NB_ELEMENTS ];// = malloc(cols * rows * sizeof(CloudPoint *));//           [ 7500 ];
// CloudPoint cloudWithObject  [ ARRAY_NB_ELEMENTS ];
// CloudPoint cloudObject      [ ARRAY_NB_ELEMENTS ];

//uint16_t distances [ ARRAY_NB_ELEMENTS ];
uint16_t distances [ int( (h_stop_angle - h_start_angle)/horizServoStep * (v_stop_angle - v_start_angle)/vertServoStep ) ];

//CloudPoint * cloud [] = malloc( cols * rows * sizeof( CloudPoint ) );

void setup() {

/* Should help, in case we need malloc (Not working as is though)
  *cloud = malloc(cols * rows * sizeof(CloudPoint *));
    for (int i=0; i<(cols * rows); i++) 
    {
        //elementsInLine[i] = malloc(FILE_LINE_SIZE);
        //strcpy( elementsInLine[i], "-" ); // Filling the array with our empty character
    }
*/

  Serial.begin( 115200 );   // Intialize terminal serial port
  delay(20);               // Give port time to initalize

  Serial.println( "TFMPlus 3D Scanning" );
  printf( "%zu %zu %zu\n", sizeof(distances), sizeof(uint16_t), sizeof(distances)/sizeof(distances[0]) );
  
  scannerInit( horizServoPin, vertServoPin );//100 360

  testLidar();
  

// /*

  Serial.print( "--- Nb of values for distances array " );
  Serial.println( int( (h_stop_angle - h_start_angle)/horizServoStep * (v_stop_angle - v_start_angle)/vertServoStep ) );

  Point3D origin, aimedAtPoint;
  origin.x = origin.y = origin.z = aimedAtPoint.x = aimedAtPoint.y = aimedAtPoint.z = 0.0;
  uint16_t retrievedDistance;
  
  Serial.println( "\n///// Initiating test box boudaries");
  // TOCHECK
  scannerTestBox3Dimensions( 92, 60, 92, 180, 0, 60 );
  // OLD VERSION
  retrievedDistance = testBoxDimension( h_x_angle, v_x_angle, aimedAtPoint );
  print3DPoint( &aimedAtPoint );
  Serial.println( "----- X : retrieved distance " + String( retrievedDistance ) + " computed Distance " + String( computeDistance( origin, aimedAtPoint ) ) );
  delay( 5000 );
/*  
  retrievedDistance = testBoxDimension( h_y_angle, v_y_angle, aimedAtPoint );
  print3DPoint( &aimedAtPoint );
  Serial.println( "----- Y : retrieved distance " + String( retrievedDistance ) + " computed Distance " + String( computeDistance( origin, aimedAtPoint ) ) );
  delay( 5000 );
  retrievedDistance = testBoxDimension( h_z_angle, v_z_angle, aimedAtPoint );
  print3DPoint( &aimedAtPoint );
  Serial.println( "----- Z : retrieved distance " + String( retrievedDistance ) + " computed Distance " + String( computeDistance( origin, aimedAtPoint ) ) );
  delay( 5000 );
  Serial.println( "///// Terminating test box boudaries\n");
// */
}

//void loop() 
//{

//}

void loop() {

  if( getIsScanning() == 0 ) {
//  if( is_scanning == 0 ) {
    int reading = digitalRead( CMD_BTN_PIN );

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    
    if ( ( millis() - lastDebounceTime ) > debounceDelay ) {

      if ( reading != buttonState ) 
      {
          buttonState = reading;
          
          // only toggle next step if the new button state is HIGH
          if ( buttonState == HIGH ) {
            nbBtnClick += 1;
            //Serial.println( nbBtnClick );
            
            if( nbBtnClick == 1 ) 
            {
              //scannerTestBoxDimensions();

              Serial.println( "----- Scanning empty scene - " + String( nbBtnClick ) );
  //            is_scanning = true;
              setIsScanning( true );
              scanning( distances, false, true );
              Serial.println( "----- Scanning empty scene - DONE - is_scanning " + String( getIsScanning() ) );
              //Point3D emptyScene = computeBaseHeight( distances );
              //Serial.println( "---- Distance to limits : " + String( emptyScene.x ) + " " + String( emptyScene.y ) + " " + String( emptyScene.z ) );
              
              // getGroundDistance
              debounceDelay = 50;
              
            }
            else if( nbBtnClick == 2 ) 
            {
              Serial.println( "----- Scanning scene with objects - "  + String( nbBtnClick ) );
              //is_scanning = true;
              setIsScanning( true );
              //scanning( OUTPUT_JS, OUTPUT_SERIAL, "with_objects", withObjectSavePath, cloudWithObject, false );
              scanning( distances, true, true );
            }

            if ( nbBtnClick >= MAX_PROG_STEP ) 
            {
              nbBtnClick = 0;
            }
          }
        }

    }

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState = reading;

  }

}
