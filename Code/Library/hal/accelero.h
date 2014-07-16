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
 * \file accelero.h
 *
 * This file define the accelerometer's data type
 */


#ifndef ACCEL_H_
#define ACCEL_H_

typedef struct
{
	float data[3];
	float temperature;
	float last_update;
} accelero_data_t;

#endif /* ACCEL_H_ */