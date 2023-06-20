/**
 * 
 * @file computations.h
 * @author Loïc Reboursière - UMONS, CLick (loic.reboursiere@umons.ac.be)
 * @brief This code is involved in the development of a 3D motorized LiDAR scanner based on servotomors. 
 * This file includes all the structures and function that compute the cloud points and the various distances and measures of the object.
 * This code, in the vast majority, needs to be tested in deep, as it has barely been integrated in the main code.
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

#ifndef COMPUTATIONS_H
#define COMPUTATIONS_H

#include <Arduino.h>
#include "scanner.h"

extern  const uint8_t rows;
extern  const uint8_t cols;

/** @struct Dimension
 *  @brief This structure is used to store 3D dimensions of an object.
 *  @var Dimension::height 
 *  Height of the object.
 *  @var Dimension::width 
 *  Width of the object.
 *  @var Dimension::depth 
 *  Depth of the object.
 */
typedef struct Dimension
{
    int height;
    int width;
    int depth;
} Dimension;

/** @struct GroundsHeights
 *  @brief This structure is used to store the different "grounds" of a specific space, 
 * the actual ground of a support that can be added to the scene.
 *  @var GroundsHeights::distanceToGround 
 *  The distance from the sensor position to the actual ground.
 *  @var GroundsHeights::distanceToSupport 
 *  The distance from the sensor position to the support position.
 */
typedef struct GroundsHeights
{
    float distanceToGround  = -1.;
    float distanceToSupport = -1.;
} GroundsHeights;

/**
 * @brief Subtract two cloud points (CloudPoint []) obtained from Lidar scanner
 * 
 * @param background Cloud point of the background, without object
 * @param withObject Cloud point of the background with the object
 * @param result Cloud point representing the object aka the result of the subtraction of the withObject and background cloud points
 * @param threshold Theshold 
 */
void subtractCloudPoint( CloudPoint background[], CloudPoint withObject[], CloudPoint result[], int16_t threshold )
{
    if( sizeof( background ) == sizeof( withObject ) == sizeof( result ))
    {
        int nbElements = sizeof( *background ) / sizeof( CloudPoint );
        for( int i = 0; i < nbElements ; i++ )
        {
            if( threshold > 0 )
            {
                if( withObject[ i ].scanCoord.distance - background[ i ].scanCoord.distance < threshold )
                {
                    result[ i ].scanCoord.distance = 0;
                }
                else
                {
                    result[ i ].scanCoord.distance = 
                        withObject[ i ].scanCoord.distance - 
                        background[ i ].scanCoord.distance;
                }
            }
            else
                result[ i ].scanCoord.distance = 
                    withObject[ i ].scanCoord.distance - 
                    background[ i ].scanCoord.distance;
        }   
    }
    else
    {
        printf( "Length of arrays don't match, check number of elements" );
    }
    
}

/**
 * @fn float computeDistance ( Point3D start, Point3D stop ) 
 *
 * @brief Compute distance between two Point3D structures
 * 
 * @param start Cloud point of the background, without object
 * @param stop Cloud point of the background with the object
 * 
 * @return distanceBtwStartStop The actual distance between the two points. 
 */
float computeDistance ( Point3D start, Point3D stop ) 
{
    float distanceBtwStartStop = sqrt( pow( ( stop.x - start.x ), 2 ) 
                                     + pow( ( stop.y - start.y ), 2 ) 
                                     + pow( ( stop.z - start.z ), 2 ) 
                                    );

    return distanceBtwStartStop;
}



// TODO : algo détection sol et support
// Soit deux passes : la première pour le sol (le plus bas) la deuxième pour le support (le plsu bas après le sol)
// Soit mémoire de qq échantillons et trouver les valeurs identiques et dont les index se suivent

/**
 * @fn GroundsHeights groundAndSupportDetection2Passes( Point3D cloud [] )
 * 
 * @brief Compute the distances from the sensor to the ground and to the support object (e.g. table, box, etc.), if present, 
 * from the background cloud point variable. The cloud point is represented by an array of 3DPoint structures.
 * 
 * @param cloud Cloud point of the background, without object
 * 
 * @return heights GroundsHeights structure containing both distances 
 */
GroundsHeights groundAndSupportDetection2Passes( Point3D cloud [] )
{

    float z_max = 0, z_min = DISTANCE_MAX;
    int idx_z_max = -1;

    GroundsHeights heights;

    for( int i = 0; i < ARRAY_NB_ELEMENTS; i++ )
    {
        if( cloud[ i ].z > z_max )
        {
            z_max = cloud[ i ].z;
            idx_z_max = i;
        }
    }
    
    for( int i = 0; i < ARRAY_NB_ELEMENTS; i++ )
    {
        if( abs( heights.distanceToGround - cloud[ i ].z ) < z_min )
        {
            z_min = cloud[ i ].z;
        }
    }

    heights.distanceToGround = z_max;
    heights.distanceToSupport = z_min;

    return heights;

}

/**
 * @fn GroundsHeights computeHeights( uint16_t distances [] )
 * 
 * @brief Compute the distances from the sensor to the ground and to the support object (e.g. table, box, etc.), if present, 
 * from the background cloudpoint variable. The cloud point is represented by an array of unint16_t values.
 * 
 * @param distances Array of distances gathered from the scan of the background, without object.
 * 
 * @return heights GroundsHeights structure containing both distances. 
 */
GroundsHeights computeHeights( uint16_t distances [] )
{

    float x = 0, y = 0, z = 0;
    float z_min = DISTANCE_MAX, z_max = 0, x_min = DISTANCE_MAX, x_max = 0, y_min = DISTANCE_MAX, y_max = 0;
    int idx_z_min = -1, idx_z_max = -1, idx_x_min = -1, idx_x_max = -1, idx_y_min = -1, idx_y_max = -1;
    int height = -1;
    uint8_t h_index = -1, v_index = -1, h_angle = -1, v_angle = -1;

    Point3D distances3D;
    GroundsHeights heights;

    Serial.println( "------ Start Base computation" );


    for( int i = 0; i < ARRAY_NB_ELEMENTS ; i++ )
    {
        if ( distances[ i ] > 0 )
        {
            oneLineIndex2HorizAndVertIndex( i, &h_index, &v_index );
            pointIndexes2PointAngles( h_index, v_index, &h_angle, &v_angle ); // Still useful ?
            computeXYZ( &x, &y, &z, distances[ i ], h_angle, v_angle );

            if( z > z_max )
            {
                z_max = z;
                idx_z_max = i;
            }

        }
    }

    Serial.println( "------ Stop Base computation" );

    distances3D.x = x_max - x_min;
    distances3D.y = y_max - y_min;
    distances3D.z = z_max - z_min;

    Serial.println("x_min " + String( x_min ) + " (idx : " + String( idx_x_min ) + ")" );
    Serial.println("x_max " + String( x_max ) + " (idx : " + String( idx_x_max ) + ")" );
    Serial.println("y_min " + String( y_min ) + " (idx : " + String( idx_y_min ) + ")" );
    Serial.println("y_max " + String( y_max ) + " (idx : " + String( idx_y_max ) + ")" );
    Serial.println("z_min " + String( z_min ) + " (idx : " + String( idx_z_min ) + ")" );
    Serial.println("z_max " + String( z_max ) + " (idx : " + String( idx_z_max ) + ")" );
    

    float distanceBtwStartStop = sqrt( pow( ( x_max - x_min ), 2 ) 
                                      + pow( ( y_max - y_min ), 2 ) 
                                      + pow( ( z_max - z_min ), 2 ) 
                                      );

    Serial.println( "--- distanceBtwStartStop " + String( distanceBtwStartStop ) );


    return heights;
}



/**
 * @fn int computeObjectHeight( uint16_t distances [] )
 * 
 * @brief Compute the height on the object based on the array of distances representing the cloud point of the object 
 * (obtained after the subtraction of the two scanned cloud points).
 * 
 * @param distances Array of distances representing the object, result of the subtraction of the two scans.
 * 
 * @return height Height of the object. 
 */
int computeObjectHeight( uint16_t distances [] )
{
    // Get min and max on the z axis of the 3D corrdinates 
    float x = 0, y = 0, z = 0, z_min = DISTANCE_MAX, z_max = 0;
    int idx_z_min = -1, idx_z_max = -1;
    int height = -1;
    uint8_t h_index = -1, v_index = -1, h_angle = -1, v_angle = -1;
    Point3D lowestHeightPoint, highestHeightPoint;

    // find the two CloudPoints where z is minimum and z is max
    printf( "computeObjectHeight -- Starting Height computation\n" );
    for( int i = 0; i < ARRAY_NB_ELEMENTS ; i++ )
    {
        // TODO : 
        // * Retrieve h and v angle from i
        // * compute xyz
        // * compare z in order to have min_z and max_z
        // * Compute distance between these two z

        // We only look for highest and lowest z on distances that are bigger than 0
        // distance = 0 means there is nothing at this specific point (i.e computing the 
        // difference between points from object's and background clouds give something
        // below or equal to CLOUD_SUBTRACT_THRESHOLD)
        if ( distances[ i ] > 0 )
        {
            oneLineIndex2HorizAndVertIndex( i, &h_index, &v_index );
            pointIndexes2PointAngles( h_index, v_index, &h_angle, &v_angle );
            computeXYZ( &x, &y, &z, distances[ i ], h_angle, v_angle );

            if( z < z_min )
            {
                z_min = z;
                lowestHeightPoint.x = x;
                lowestHeightPoint.y = y;
                lowestHeightPoint.z = z;
                idx_z_min = i;
            }
            else if( z > z_max )
            {
                z_max = z;
                highestHeightPoint.x = x;
                highestHeightPoint.y = y;
                highestHeightPoint.z = z;
                idx_z_max = i;
            }
        }
    }
    printf( "computeObjectHeight -- Finished scanning throught distances\n" );
    // TODO : replace this by computing distance between highestHeightPoint and lowestHeightPoint
    height = distances[idx_z_max] - distances[idx_z_min];
    printf( "computeObjectHeight -- distances[%i] - distances[%i] = %i - %i = %i\n", idx_z_max, idx_z_min, distances[ idx_z_max ], distances[ idx_z_min ], height );
    
    height = sqrt( pow( highestHeightPoint.x - lowestHeightPoint.x, 2 ) 
                    - pow( highestHeightPoint.y - lowestHeightPoint.y, 2 ) 
                    - pow( highestHeightPoint.z - lowestHeightPoint.z, 2 )
                );

    printf( "computeObjectHeight -- height with full distance %i\n", height );

    return height;
}

/**
 * @fn void subtractMatrices( int * bg, int * img, int * result )
 * 
 * @brief Subtract the two cloud points obtained after each scan.
 *
 * @param[in]  bg A pointer to the background cloudpoint.
 * @param[in]  img A point to the scene with object cloudpoint.
 * @param[out] result The result of the subtraction of the two cloudpoint.
 *
 * @return Nothing
 */
void subtractMatrices( int * bg, int * img, int * result )
{
    for( int i = 0; i < rows; i++ )
    {
        for( int j = 0; j < cols; j++ )
        {
            *( ( result + i * cols) + j) = *( ( bg + i * cols ) + j ) - *( ( img + i * cols ) + j );
        }
    }
}


#endif