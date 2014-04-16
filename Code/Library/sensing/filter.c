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
*	\file filter.c
*	\brief Source file of the filtering module.
*
*	This module provides low-pass filters
*	This file contains all the internal variables and functions of the module.
*
*/

//----------
// Includes
//----------

#include "filter.h"


//---------------------------------
// Public functions implementation
//---------------------------------

/*!
*	This function applies a simple low-pass filter
*
*	@param valueToFilter The value that will be filtered
*	@param valueInput The value to use for the difference
*	@param timeConstantRez The time constant for the low-pass (LP) filter is given reziprocally.
*	@param dt The delta time
*/
void filter_LowPassi(float * valueToFilter, int valueInput, float timeConstantRez, float dt)
{
	*valueToFilter = ((float)valueInput - *valueToFilter) * dt * timeConstantRez + *valueToFilter;	
}	

/*!
*	This function applies a simple low-pass filter
*
*	@param valueToFilter The value that will be filtered
*	@param valueInput The value to use for the difference
*	@param timeConstantRez The time constant for the low-pass (LP) filter is given reziprocally.
*	@param dt The delta time
*/
void filter_LowPassui(float * valueToFilter, unsigned int valueInput, float timeConstantRez, float dt)
{
	*valueToFilter = ((float)valueInput - *valueToFilter) * dt * timeConstantRez + *valueToFilter;	
}

/*!
*	This function applies a simple low-pass filter
*
*	@param valueToFilter The value that will be filtered
*	@param valueInput The value to use for the difference
*	@param timeConstantRez The time constant for the low-pass (LP) filter is given reziprocally.
*	@param dt The delta time
*/	
void filter_LowPassl(float * valueToFilter, long valueInput, float timeConstantRez, float dt)
{
	*valueToFilter = ((float)valueInput - *valueToFilter) * dt * timeConstantRez + *valueToFilter;	
}	

/*!
*	This function applies a simple low-pass filter
*
*	@param valueToFilter The value that will be filtered
*	@param valueInput The value to use for the difference
*	@param timeConstantRez The time constant for the low-pass (LP) filter is given reziprocally.
*	@param dt The delta time
*/
void filter_LowPassf(float * valueToFilter, float valueInput, float timeConstantRez, float dt)
{
	*valueToFilter = (valueInput - *valueToFilter) * dt * timeConstantRez + *valueToFilter;	
}

