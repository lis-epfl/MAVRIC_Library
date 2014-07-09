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
	uint16_t raw_data[3];		///< Raw values from the
	float bias[3];			///< bias factor
	float scale[3];			///< scale factor
	float data[3];			///< computed values, using raw_data - bias factor and scale factor
} compass_data_t;

#endif /* COMPASS_H_ */