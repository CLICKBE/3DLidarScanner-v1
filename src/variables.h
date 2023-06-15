/**
 * 
 * @file variable.h
 * @author Loïc Reboursière - UMONS, CLick (loic.reboursiere@umons.ac.be)
 * @brief This code is involved in the development of a 3D motorized LiDAR scanner based on servomotors.
 * This file contains global variables that are common to different part of the code.
 * To simplify the modification by a user of this code, it has been decided to gather those variables in one place.
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

#ifndef VARIABLES_H
#define VARIABLES_H

const uint8_t h_start_angle = 0;// Start angle for horizontal displacement of LiDAR sensor
const uint8_t h_stop_angle = 92;//100 140;// Stop angle for horizontal displacement of LiDAR sensor
const uint8_t v_start_angle = 150;//110;// Start angle for vertical displacement of LiDAR sensor
const uint8_t v_stop_angle = 150;//160;// Stop angle for vertical displacement of LiDAR sensor
// Horizontal aperture [] - Vertical aperture [110;180]
// For testing boundaries
const uint8_t h_x_angle = 110;//20
const uint8_t v_x_angle = 180 ;
const uint8_t h_y_angle = 110; 
const uint8_t v_y_angle = 90; 
const uint8_t h_z_angle= 20;
const uint8_t v_z_angle = 210;

const uint8_t horizServoStep = 1;// Step in horizontal displacement of LiDAR sensor
const uint8_t vertServoStep = 1;// Step in vertical displacement of LiDAR sensor

const uint16_t delayOneStepServo = 5;// Delay let to the servo to make one horizontal step
const uint16_t delayBacktoStartAngle = 500;// Delay let to the servo to go back to h_start_angle horizontal position

// cols * rows = (h_stop_angle - h_start_angle)/horizServoStep * (v_stop_angle - v_start_angle)/vertServoStep
#define ARRAY_NB_ELEMENTS 2400//4900//6300 
//20*120
#define DISTANCE_MAX 500

#define CLOUD_SUBTRACT_THRESHOLD 0.01

#endif