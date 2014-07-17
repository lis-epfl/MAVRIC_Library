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
	float data[3];
	float temperature;
	float last_update;
} gyro_data_t;

#endif /* GYRO_H_ */