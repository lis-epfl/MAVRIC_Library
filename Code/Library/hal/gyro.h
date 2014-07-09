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
 * \file gyro.h
 *
 * This file define the gyroscope's data type
 */


#ifndef GYRO_H_
#define GYRO_H_

typedef struct
{
	int8_t temperature;				///< Define the temperature of the sensor
	float raw_data[3];		///< Raw values from the 
	float bias[3];			///< bias factor 
	float scale[3];			///< scale factor
	float data[3];			///< computed values, using raw_data - bias factor and scale factor
} gyro_data_t;

#endif /* GYRO_H_ */