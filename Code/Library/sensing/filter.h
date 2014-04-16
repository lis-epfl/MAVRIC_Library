/*
	Copyright (C) 2008-2011. LIS Laboratory, EPFL, Lausanne
	Licensed to senseFly LLC in 2011
*/
/*
	Copyright (c) 2012 senseFly
	Author: Antoine Beyeler

	All rights reserved. Do not distribute.

	Laboratory of Intelligent Systems (LIS) internal use only.
	Written authorization by SenseFly required for distribution or use outside the LIS.
*/
/*!
*	\file filter.h
*	\brief Header file of the filtering module.
*/

#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif


//----------------------------
// Public function prototypes
//----------------------------

//! This function applies a low-pass filter (int type)
void filter_LowPassi(float * valueToFilter, int valueInput, float timeConstantRez, float dt);

//! This function applies a low-pass filter (unsigned int type)
void filter_LowPassui(float * valueToFilter, unsigned int valueInput, float timeConstantRez, float dt);

//! This function applies a low-pass filter (long type)
void filter_LowPassl(float * valueToFilter, long valueInput, float timeConstantRez, float dt);

//! This function applies a low-pass filter (float type)
void filter_LowPassf(float * valueToFilter, float valueInput, float timeConstantRez, float dt);


#ifdef __cplusplus
}
#endif

#endif
