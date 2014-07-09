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
	float raw_data[3];		///< Raw values from the
	float bias[3];			///< bias factor
	float scale[3];			///< scale factor
	float data[3];			///< computed values, using raw_data - bias factor and scale factor
} accelero_data_t;

#endif /* ACCEL_H_ */