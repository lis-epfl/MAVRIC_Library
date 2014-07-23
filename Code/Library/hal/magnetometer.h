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
 * \file magnetometer.h
 *
 * This file define the compass's data type
 */


#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

typedef struct
{
	float data[3];
	float temperature;
	float last_update;
} magnetometer_t;

#endif /* MAGNETOMETER_H_ */