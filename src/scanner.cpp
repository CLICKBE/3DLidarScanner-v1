/**
 * 
 * @file scanner.cpp
 * @author Loïc Reboursière - UMONS, CLick (loic.reboursiere@umons.ac.be)
 * @brief This code is involved in the development of a 3D motorized LiDAR scanner based on servotomors. 
 * This file includes the different 3D LiDAR scanner available functions implementations.
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

#include "scanner.h"


bool is_scanning, is_scanning_object, is_sending_center, is_line_finished;

int currentHorizontalAngle, currentVerticalAngle;

int16_t distance, strength, flux, temp;

extern Servo myservo_horiz; 
extern Servo myservo_vert;
//extern int horizServoPin;
//extern int vertServoPin;

extern TFMPlus tfmS;



int16_t testBoxDimension( uint8_t h_angle, uint8_t v_angle, Point3D &aimedAtPoint )
{

  setPosition( h_angle, v_angle, true );
  delay( 1000 );
  tfmS.getData( distance, flux, temp);
  delay( 100 );
  computeXYZ( &aimedAtPoint.x, &aimedAtPoint.y, &aimedAtPoint.z, h_angle, v_angle, distance );

  return distance;
}

void scannerTestBox3Dimensions( uint8_t h_x_angle, uint8_t v_x_angle, uint8_t h_y_angle, uint8_t v_y_angle, uint8_t h_z_angle, uint8_t v_z_angle ) 
{

  Point3D start, stop, origin, aimedAtPoint;
  origin.x = origin.y = origin.z = 0;

  //uint8_t h_x_angle = 92, v_x_angle = 60, h_y_angle = 92, v_y_angle = 180, h_z_angle= 0, v_z_angle = 60;

  myservo_horiz.write( h_x_angle );
  myservo_vert.write ( v_x_angle );
  delay( 1000 );
  tfmS.getData( distance, flux, temp);
  computeXYZ( &aimedAtPoint.x, &aimedAtPoint.y, &aimedAtPoint.z, h_x_angle, v_x_angle, distance );
  //Serial.println( "----- X " + String( distance ) + " - " + String( computeDistance( origin, aimedAtPoint ) ) );
  //delay( 1000 );

  myservo_horiz.write( h_y_angle );
  myservo_vert.write ( v_y_angle );
  delay( 1000 );
  tfmS.getData( distance, flux, temp);
  computeXYZ( &aimedAtPoint.x, &aimedAtPoint.y, &aimedAtPoint.z, h_y_angle, v_y_angle, distance );
  //Serial.println( "----- Y " + String( distance ) + " - " + String( computeDistance( origin, aimedAtPoint ) ) );

  myservo_horiz.write( h_z_angle );
  myservo_vert.write ( v_z_angle );
  delay( 1000 );
  tfmS.getData( distance, flux, temp);
  computeXYZ( &aimedAtPoint.x, &aimedAtPoint.y, &aimedAtPoint.z, h_z_angle, v_z_angle, distance );
  //Serial.println( "----- Y " + String( distance ) + " - " + String( computeDistance( origin, aimedAtPoint ) ) );

  //float distanceBtwStartStop = computeDistance( start, stop );

  //Serial.println( "--- distanceBtwStartStop " + String( distanceBtwStartStop ) );

// sqrt((x2−x1)^2+(y2−y1)^2+(z2−z1)^2)

}

void scannerTestBoxDimensions() 
{

  Point3D start, stop;

  Serial.println( "---- Position initiale " );
  myservo_horiz.write( h_start_angle );
  myservo_vert.write ( v_start_angle );
  tfmS.getData( distance, flux, temp);
  computeXYZ( &start.x, &start.y, &start.z, distance, h_start_angle, v_start_angle );
  Serial.println( "----- Largeur " + String( distance ) );
  delay( 1000 );

  myservo_horiz.write( h_stop_angle );
  delay( 1000 );
  tfmS.getData( distance, flux, temp);
  computeXYZ( &stop.x, &stop.y, &stop.z, distance, h_stop_angle, v_start_angle );
  Serial.println( "----- Longueur " + String( distance ) );

  float distanceBtwStartStop = sqrt( pow( ( stop.x - start.x ), 2 ) 
                                    + pow( ( stop.y - start.y ), 2 ) 
                                    + pow( ( stop.z - start.z ), 2 ) 
                                    );

  Serial.println( "--- distanceBtwStartStop " + String( distanceBtwStartStop ) );

// sqrt((x2−x1)^2+(y2−y1)^2+(z2−z1)^2)

}

void scannerInit( int h_pin, int v_pin ) 
{
    is_scanning = false; 
    is_scanning_object = false;
    is_sending_center = false;
    is_line_finished = false;  

    distance = 0;
    strength = 0;
    flux = 0;
    temp = 0;

    TF_SERIAL.begin( TFMINI_BAUDRATE, SERIAL_8N1, TF_TX, TF_RX );
    tfmS.begin( &TF_SERIAL );

    #ifdef ESP32
        ESP32PWM::allocateTimer( 0 );
        ESP32PWM::allocateTimer( 1 );
        ESP32PWM::allocateTimer( 2 );
        ESP32PWM::allocateTimer( 3 );
        myservo_horiz.setPeriodHertz( 50 );      // Standard 50hz servo
        myservo_vert.setPeriodHertz( 50 );      // Standard 50hz servo
        myservo_horiz.attach( h_pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH );  
        myservo_vert.attach( v_pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH );

    #else
        myservo_horiz.attach( h_pin );  
        myservo_vert.attach( v_pin );
    #endif

    
}

void setPosition( int h_angle, int v_angle, bool verbose )
{
  currentHorizontalAngle = h_angle;
  currentVerticalAngle = v_angle;

  if( verbose )
  {
    Serial.println ( "Setting Horizontal angle at : " + String( currentHorizontalAngle ) );
    Serial.println ( "Setting Vertical angle at : " + String( currentVerticalAngle ) );
  }

  myservo_horiz.write( currentHorizontalAngle );
  myservo_vert.write ( currentVerticalAngle );


}

void testLidar () 
{
    /************* From TFMINI Plus Arduino library example ****/
  printf( "Soft reset: ");
  if( tfmS.sendCommand( SOFT_RESET, 0))
  {
      printf( "passed.\r\n");
  }
  else tfmS.printReply();

  delay(500);  // added to allow the System Rest enough time to complete

// - - Display the firmware version - - - - - - - - -
  printf( "Firmware version: ");
  if( tfmS.sendCommand( GET_FIRMWARE_VERSION, 0))
  {
      printf( "%1u.", tfmS.version[ 0 ]); // print three single numbers
      printf( "%1u.", tfmS.version[ 1 ]); // each separated by a dot
      printf( "%1u\r\n", tfmS.version[ 2 ]);
  }
  else tfmS.printReply();
  // - - Set the data frame-rate to 20Hz - - - - - - - -
  printf( "Data-Frame rate: ");
  if( tfmS.sendCommand( SET_FRAME_RATE, FRAME_20))
  {
      printf( "%2uHz.\r\n", FRAME_20);
  }
  else tfmS.printReply();

  delay(500);

}

void updateServoPosition() {

  int delayHoriztalServo = delayOneStepServo;
  int delayVerticalServo = 30;

  if (currentHorizontalAngle + horizServoStep <= (h_stop_angle - horizServoStep) ) 
    currentHorizontalAngle += horizServoStep;
  else 
  {
    currentHorizontalAngle = h_start_angle;
    delayHoriztalServo = delayBacktoStartAngle;
    is_line_finished = true;
  }

  if ( is_line_finished )   
  {
      
    if ( currentVerticalAngle + vertServoStep <= (v_stop_angle - vertServoStep) ) 
    {
      currentVerticalAngle += vertServoStep;
      is_line_finished = false;
    }
    else 
    {
      currentVerticalAngle = v_start_angle;
      delayVerticalServo = delayBacktoStartAngle;
      is_scanning = false;
    }
    myservo_vert.write( currentVerticalAngle );
    delay( delayVerticalServo );
  }

  myservo_horiz.write( currentHorizontalAngle );
  delay( delayHoriztalServo );
}  


void sendDataSerial ( float x, float y, float z, int horizontalAngle, int verticalAngle, uint16_t distance ) 
{

  Serial.print( x );
  Serial.write( ' ');
  Serial.print( y );
  Serial.write( ' ');
  Serial.println( z );
  Serial.write( 9 );
  Serial.print( horizontalAngle );
  Serial.write( 9 );
  Serial.print( verticalAngle );
  Serial.write( 9 );
  Serial.println(distance);

}



void computeCloudXYZ( uint16_t distances [] )
{
  uint8_t h_index = -1, v_index = -1, h_angle = -1, v_angle = -1;
  float x, y, z;

  for( int i = 0; i < ARRAY_NB_ELEMENTS; i++ )
  {
    oneLineIndex2HorizAndVertIndex( i, &h_index, &v_index );
    pointIndexes2PointAngles( h_index, v_index, &h_angle, &v_angle );
    computeXYZ
    ( 
      &x,
      &y,
      &z,
      h_index,
      v_index,
      distances[ i ]
    );
  }
}

void computeXYZ( float *x, float *y, float *z, int hAngleDegree, int vAngleDegree, int16_t distance ) {

  float pitchf = hAngleDegree * M_PI / 180;
  float yawf = vAngleDegree * M_PI / 180;

  *x = distance * -sin( yawf ) * cos( pitchf );
  *y = distance * sin( yawf ) * sin( pitchf );
  *z = distance * cos( pitchf );

}



void scanning( uint16_t distances [], bool subtract, bool output_serial ) 
{

  float x = -1., y = -1., z = -1.;

  if( is_scanning ) 
  {

    while ( is_scanning ) 
    {
      uint8_t horizontalIndex, verticalIndex; 

      pointAngles2PointIndexes( currentHorizontalAngle, currentVerticalAngle, &horizontalIndex, &verticalIndex );

      if( !tfmS.getData( distance, flux, temp) ) // Get data from the device.
      {
        tfmS.printFrame();  // display the error and HEX dataa
      }
              
      if( subtract ) 
      { 
        int oneLineArrayId = getOneLineArrayIndex( horizontalIndex, verticalIndex );
        uint16_t subtraction = abs ( distances[ oneLineArrayId ] - distance );
        if( subtraction <= CLOUD_SUBTRACT_THRESHOLD )
        {
          distances[ oneLineArrayId ] = 0;
        }
        else 
        {
          distances[ oneLineArrayId ] = subtraction;
        }
  
        if( output_serial ) 
        {
          computeXYZ( &x, &y, &z, currentHorizontalAngle, currentVerticalAngle, distance );
          Serial.println( "Subtracted" );
          sendDataSerial( x, y, z, currentHorizontalAngle, currentVerticalAngle, distances[ oneLineArrayId ] );
        }
        
      }
      else
      {
        distances[ getOneLineArrayIndex( horizontalIndex, verticalIndex ) ] = distance;
        if( output_serial ) 
        {
          computeXYZ( &x, &y, &z, horizontalIndex, verticalIndex, distance );
          Serial.println( "FirstScan" );
          sendDataSerial( x, y, z, horizontalIndex, verticalIndex, distance );
        }
      }

      
      #if DEBUG
        printData();   // display distance,
      #endif

      delay( 2 );
      
      updateServoPosition(); 

    }
  }
}


void pointAngles2PointIndexes ( uint8_t h_angle, uint8_t v_angle, uint8_t * h_index, uint8_t * v_index )
{
  *h_index = ( h_angle - h_start_angle ) / horizServoStep;
  *v_index = ( v_angle - v_start_angle ) / vertServoStep;
}


void pointIndexes2PointAngles ( uint8_t h_index, uint8_t v_index, uint8_t * h_angle, uint8_t * v_angle )
{
  *h_angle = h_index * horizServoStep + h_start_angle;
  *v_angle = v_index * vertServoStep  + v_start_angle;
}


uint16_t getOneLineArrayIndex( uint8_t h_index, uint8_t v_index )
{
  return uint16_t( ( v_index * cols ) + h_index );
}

void oneLineIndex2HorizAndVertIndex( int i, uint8_t * h_index, uint8_t * v_index )
{
  *h_index = i % cols;
  *v_index = i / cols;
}


void setIsScanning ( bool value ) { is_scanning = value; };
void setIsScanningObject ( bool value ) { is_scanning_object = value; };
void setIsSendingCenter ( bool value ) { is_sending_center = value; };
void setIsLineFinished ( bool value ) { is_line_finished = value; }; 

bool getIsScanning () { return is_scanning; };
bool getIsScanningObject () { return is_scanning_object; };
bool getIsSendingCenter () { return is_sending_center; };
bool getIsLineFinished () { return is_line_finished; };
