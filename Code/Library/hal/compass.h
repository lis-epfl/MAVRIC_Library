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
 * \file compass.h
 *
 * This file define the compass's data type
 */


#ifndef COMPASS_H_
#define COMPASS_H_

typedef struct
{
	float data[3];
	float temperature;
	float last_update;
} compass_data_t;

#endif /* COMPASS_H_ */