/*
 * orca.h
 *
 *  Created: 22.10.2013 09:57:24
 *  Author: ndousse
 */ 


#ifndef ORCA_H__
#define ORCA_H__

#include "neighbor_selection.h"

#include <stdbool.h>
#include <stdint.h>

#define ORCA_TIME_STEP_MILLIS 10.0

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

void init_orca();

void computeNewVelocity();

bool linearProgram1(plane_t planes[], int8_t index, line_t line, float maxSpeed, float OptimalVelocity[], float NewVelocity[], bool directionOpt);
bool linearProgram2(plane_t planes[], int8_t ind, float maxSpeed, float OptimalVelocity[], float NewVelocity[], bool directionOpt);
float linearProgram3(plane_t planes[], float OptimalVelocity[], float maxSpeed, float NewVelocity[], bool directionOpt);
void linearProgram4(plane_t planes[], int8_t ind, float maxSpeed, float NewVelocity[]);

#endif ORCA_H__