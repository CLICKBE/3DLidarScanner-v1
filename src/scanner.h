/**
 * 
 * @file scanner.h
 * @author Loïc Reboursière - UMONS, CLick (loic.reboursiere@umons.ac.be)
 * @brief This code is involved in the development of a 3D motorized LiDAR scanner based on servotomors. 
 * This file includes the different 3D LiDAR scanner available functions definitions.
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

#ifndef SCANNER_H
#define SCANNER_H

#include <Arduino.h>

#ifdef ESP32
 #include <ESP32Servo.h>
#else
 #include <Servo.h>
#endif

#include <TFMPlus.h>
#include "variables.h"
//#include "computations.h"

#define TFMINI_BAUDRATE 115200
#define TF_SERIAL Serial1
#define TF_RX 5
#define TF_TX 4

extern const uint8_t cols;
extern const uint8_t rows;

typedef struct PointAngles
{
  int16_t horizAngle;
  int16_t vertAngle;
} PointAngles;

typedef struct PointIndexes
{
  int16_t horizIdx;
  int16_t vertIdx;  
} PointIndexes;

typedef struct Point3D
{
  float x;
  float y;
  float z;

} Point3D;

typedef struct ScanPoint
{
  int16_t horizIdx;
  int16_t vertIdx;
  int16_t distance;
} ScanPoint;

typedef struct CloudPoint
{
    ScanPoint scanCoord;
    Point3D coord3D;
} CloudPoint;

/**
 * @fn int16_t testBoxDimension( uint8_t h_angle, uint8_t v_angle, Point3D &aimedAtPoint )
 * @brief This function is used as a test to check if the sensor picks up the right distance. 
 * It's meant to be used with known distances which can be compared to sensor's measurement.
 * This function makes the motor-driven 3D arm go to a certain position defined by h_angle and v_angle 
 * and return the distance that the LiDAR sensor measured. This function also computes the 3D coordinates 
 * of the position throught the aimedAtPoint Point3D pointer.
 * 
 * @param h_angle Horizontal angle to which the pantilt is set.
 * @param v_angle Vertical angle to which the pantilt is set.
 * @param aimedAtPoint Pointer to the Point3D object containing the computed 3D coordinates.
 * @return Distance measured by LiDAR.
 */
int16_t testBoxDimension( uint8_t h_angle, uint8_t v_angle, Point3D &aimedAtPoint );

/**
 * @fn void scannerTestBox3Dimensions( uint8_t h_x_angle, uint8_t v_x_angle, uint8_t h_y_angle, uint8_t v_y_angle, uint8_t h_z_angle, uint8_t v_z_angle )
 * @brief Triple uses of the int16_t testBoxDimension( uint8_t h_angle, uint8_t v_angle, Point3D &aimedAtPoint ) function
 * in order to retrieve the width, height and depth of the surrounding space of the scanner. As for testBoxDimension function
 * the retrieved distances are meant to be compared to hand-measured width, height and depth of the surrounding space.
 * The given angles should therefore correspond to the 3D arm relevant positions to measure width, height and depth of 
 * the surrounding space.
 * @param h_x_angle Horizontal angle pointing at maximum distance along x axis
 * @param v_x_angle Vertical angle pointing at maximum distance along x axis
 * @param h_y_angle Horizontal angle pointing at maximum distance along y axis
 * @param v_y_angle Vertical angle pointing at maximum distance along y axis
 * @param h_z_angle Horizontal angle pointing at maximum distance along z axis
 * @param v_z_angle Vertical angle pointing at maximum distance along z axis
 */
void scannerTestBox3Dimensions( uint8_t h_x_angle, uint8_t v_x_angle, uint8_t h_y_angle, uint8_t v_y_angle, uint8_t h_z_angle, uint8_t v_z_angle );

/**
 * @brief 
 * 
 */
void scannerTestBoxDimensions();

/**
 * @brief Initialise all variables.
 * 
 */
void scannerInit( int h_pin, int v_pin );

/**
 * @brief Set the Position object
 * 
 * @param h_angle The horizontal angle at which the pantilt must go.
 * @param v_angle The vertical angle at which the pantilt must go.
 * @param verbose Whether or not to display info about the reached angles.
 */
void setPosition( int h_angle, int v_angle, bool verbose );

/**
 * @brief Check several elements with Lidar sensor.
 * 
 */
void testLidar();

/**
 * @brief Increment Servo motors positions to next step (horizServoStep or vertServoStep) and line.
 * When hitting the end of the sweep, is_scanning is set to false.
 */
void updateServoPosition();  

/**
 * @brief Print to Serial monitor (x, y) coordinates if a distance is retrieved by the LiDar sensor.
 * 
 */
void printData ();

/**
 * @brief Print the element constituting of a specific ScanPoint.
 * 
 * @param sp A pointer to the ScanPoint structure from which to print the data.
 */
void printScanPoint ( ScanPoint * sp );

/**
 * @brief Print x, y, z coordinates of a Point3D structure
 * 
 * @param 3dp A pointer to the Point3d structure from which to print the data.
 */
void print3DPoint ( Point3D * point );

/**
 * @brief Print fields of a specific CloudPoint structure
 * 
 * @param cp A pointer to the CloudPoint structure from which to print the data.
 * @param index Index of the structure in the points of Cloud.
 */
void printCloudPoint ( CloudPoint * cp, int index );

/**
 * @brief Send data through serial port. The format is "distance currentHorizontalPoint currentVerticalPoint"
 * Hardware or software willing to receive those data must be setup at TFMINI_BAUDRATE speed.
 * 
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @param z The z coordinate.
 * @param horizontalAngle The horizontal angle the scanner is at.
 * @param verticalAngle The vertical angle the scanner is at.
 * @param distance The distance measured at that specific point.
 */
void sendDataSerial ( float x, float y, float z, int horizontalAngle, int verticalAngle, uint16_t distance );

/**
 * @brief Convert horiz_step_idx, vert_step_idx and distance of a cloud point to (x, y, z) coordinates.
 * The function completes CloudPoint structures by computing structures' Point3D from the ScanPoint.
 * @param cloud Array of CloudPoint comming from a scan of the LiDAR.
 * 
 */
void computeCloudXYZ( CloudPoint cloud [] );


/**
 * @brief Compute XYZ coordinates of each element of distances [] presenting a positive value.
 * @param distances Array of distances coming from a LiDAR scanning phase.
 * @param cloud Array of computed XYZ 3D points corresponding to the initial distances array.
 */
void computeCloudXYZ( uint16_t distances [], Point3D cloud []);

/**
 * @brief Convert horiz_step_idx, vert_step_idx and distance to (x, y, z) coordinates.
 * Computation comes from : https://github.com/bitluni/3DScannerESP8266
 * And https://www.youtube.com/watch?v=vwUGPjQ_5t4
 * 
 * @param x The float pointer to the x coordinates
 * @param y The float pointer to the y coordinates
 * @param z The float pointer to the z coordinates
 * @param horizPointAngle The horizontal angle the scanner is at.
 * @param vertPointAngle The horizontal angle the scanner is at.
 * @param distance distance retrieved by LiDAR sensor
 */
void computeXYZ( float *x, float *y, float *z, int horizPointAngle, int vertPointAngle, int16_t distance );

/**
 * @brief Convert horiz_step_idx, vert_step_idx and distance to (x, y, z) coordinates.
 * Computation comes from : https://github.com/bitluni/3DScannerESP8266
 * And https://www.youtube.com/watch?v=vwUGPjQ_5t4
 * 
 * @param distance distance retrieved by LiDAR sensor
 * @param horizPointAngle Horizontal angle where the pantilt sweeping is
 * @param vertPointAngle Vertical index where the pantilt sweeping is
 * @return Point3D object containing the x, y, z coordinates
 */
Point3D computeXYZ( int16_t distance, int horizPointAngle, int vertPointAngle );

/**
 * @brief Convert horiz_step_idx, vert_step_idx and distance to (x, y, z) coordinates.
 * Computation comes from : https://github.com/bitluni/3DScannerESP8266
 * And https://www.youtube.com/watch?v=vwUGPjQ_5t4
 * 
 * @param sp The scanpoint representing the actual point
 */
Point3D computeXYZ( ScanPoint * sp );

/**
 * @brief Do the actual scanning and format data to the output defined by outputType.
 * 
 * @param output_js whether or not write xyz coordinates in js format. This file is placed in the SPIFFS
 * @param output_serial whether or not outputting data through serial
 * @param js_varname path to the js file saving the scanning data if js is chosen, if output_js is set to true 
 *  but no filename is given, the default value is distance.js
 * @param path Path of the output file
 */
void scanning( bool output_js, bool output_serial, String js_varname, char * path, CloudPoint outputArray [], bool subtract );

/**
 * @brief Do the actual scanning and format data to the output defined by outputType.
 * 
 * @param output_js whether or not write xyz coordinates in js format. This file is placed in the SPIFFS
 * @param output_serial whether or not outputting data through serial
 * @param js_varname path to the js file saving the scanning data if js is chosen, if output_js is set to true 
 *  but no filename is given, the default value is distance.js
 * @param path Path of the output file
 */
void scanning( bool output_js, bool output_serial, String js_varname, char * path, uint16_t distances [], bool subtract );

/**
 * @brief Convert horizontal and vertical angles to horizontal and vertical indexes of the scanner movement, 
 * e.g : scanning taking place between 10 and 20° horitally and 40° and 100° vertically leads to horizontal
 * and vertical indexes ranging respectively from 0 to 9 and 0 to 59.
 * This function is the opposite of pointIndexes2PointAngles.
 * 
 * @param h_angle The value of horizontal angle 
 * @param v_angle The value of vertical angle
 * @param h_index The pointer of the h_index to be computed
 * @param v_index The point to the v_index to be computed
 */
void pointAngles2PointIndexes ( uint8_t h_angle, uint8_t v_angle, uint8_t * h_index, uint8_t * v_index );

/**
 * @brief Convert horizontal and vertical index to horizontal and vertical angles of the scanner movement, 
 * e.g : scanning taking place between 10 and 20° horitally and 40° and 100° vertically leads to horizontal
 * and vertical indexes ranging respectively from 0 to 9 and 0 to 59.
 * This function is the opposite of pointAngles2PointIndexes.
 * 
 * @param h_index The value of horizontal index
 * @param v_index The value of vertical index
 * @param h_angle The pointer of the h_angle to be computed
 * @param v_angle The pointer of the v_angle to be computed
 */
void pointIndexes2PointAngles ( uint8_t h_index, uint8_t v_index, uint8_t * h_angle, uint8_t * v_angle );

void oneLineIndex2HorizAndVertIndex( int i, uint8_t * h_index, uint8_t * v_index );


/**
 *
 * @fn uint16_t getOneLineArrayIndex( uint8_t h_index, uint8_t v_index ) 
 *
 * @brief Create a 1D array representing the 2D scanning matrix to store the sensor's values.
 * 
 * @param h_index The horizontal index during the scanning operation.
 * @param v_index The vertical index during the scanning operation.
 * 
 * @return unint16_t
 */
uint16_t getOneLineArrayIndex( uint8_t h_index, uint8_t v_index );


/**
 * @brief Just print local time
 * 
 */
void printLocalTime();

// Define setters and getters
/**
 * @brief Set the is_scanning variable
 * 
 * @param value Value to assign to is_scanning
 */
void setIsScanning ( bool value );

/**
 * @brief Set the is_scanning_object object
 * 
 * @param value Value to assign to is_scanning_object
 */
void setIsScanningObject ( bool value );

/**
 * @brief Set the is_sending_center object
 * 
 * @param value Value to assign to is_sending_center
 */
void setIsSendingCenter ( bool value );

/**
 * @brief Set the is_line_finished object
 * 
 * @param value Value to assign to is_line_finished
 */
void setIsLineFinished ( bool value ); 

/**
 * @brief Get the is_scanning
 * 
 * @return is_scanning value  
 */
bool getIsScanning ();

/**
 * @brief Get the is_scanning_object value
 * 
 * @return is_scanning_object  
 */
bool getIsScanningObject ();

/**
 * @brief Get the is_sending_center value
 * 
 * @return is_sending_center value 
 */
bool getIsSendingCenter ();

/**
 * @brief Get the is_line_finished value
 * 
 * @return is_line_finished value 
 */
bool getIsLineFinished ();

#endif