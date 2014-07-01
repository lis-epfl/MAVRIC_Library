/**
 *  ORCA  
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */


#ifndef ORCA_H__
#define ORCA_H__

#include <stdbool.h>
#include <stdint.h>

#include "neighbor_selection.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ORCA_TIME_STEP_MILLIS 10.0
#define TIME_HORIZON 12.0
#define MAXSPEED 4.5
#define RVO_EPSILON 0.0001

typedef struct{
	float normal[3];
	float point[3];
}plane_t;

typedef struct{
	float direction[3];
	float point[3];
}line_t;

/**
 * \brief Initialize ORCA
 */
void init_orca(void);

/**
 * \brief Compute new velocity
 * \param OptimalVelocity
 * \param NewVelocity
 */
void computeNewVelocity(float OptimalVelocity[], float NewVelocity[]);

/**
 * \brief Linear Program
 * \param planes
 * \param index
 * \param line
 * \param maxSpeed
 * \param OptimalVelocity
 * \param NewVelocity
 * \param directionOpt 
 * \return
 */
bool linearProgram1(plane_t planes[], uint8_t index, line_t line, float maxSpeed, float OptimalVelocity[], float NewVelocity[], bool directionOpt);

/**
 * \brief Linear Program 2
 * \param planes
 * \param ind 
 * \param maxSpeed
 * \param OptimalVelocity
 * \param NewVelocity
 * \param directionOpt 
 * \return
 */
bool linearProgram2(plane_t planes[], uint8_t ind, float maxSpeed, float OptimalVelocity[], float NewVelocity[], bool directionOpt);

/**
 * \brief Linear Program 3
 * \param planes
 * \param planeSize
 * \param maxSpeed
 * \param OptimalVelocity
 * \param NewVelocity
 * \param directionOpt 
 * \return
 */
float linearProgram3(plane_t planes[], uint8_t planeSize, float OptimalVelocity[], float maxSpeed, float NewVelocity[], bool directionOpt);

/**
 * \brief Linear Program 4
 * \param planes
 * \param ind
 * \param maxSpeed
 * \param NewVelocity
 * \return
 */
void linearProgram4(plane_t planes[], uint8_t planeSize, uint8_t ind, float maxSpeed, float NewVelocity[]);

#ifdef __cplusplus
}
#endif

#endif // ORCA_H__