/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file accelerometer.h
 *
 * This file define the accelerometer's data type
 */


#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

typedef struct
{
	float data[3];
	float temperature;
	float last_update;
} accelerometer_t;

#endif /* ACCELEROMETER_H_ */