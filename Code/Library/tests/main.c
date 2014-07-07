/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file main.c
 * 
 * main to Test function
 */
 
 
#include <stdio.h>

#include "all_tests.h"
#include "delay.h"

#include "central_data.h"
#include "boardsupport.h"

/**
 * \brief main function for testing purpose
 *
 * \param argc number of argument
 * \param argv execution parameters
 *
 * \return 0 as in any main...
 */
int32_t main(int32_t argc, char **argv)
{
	
	boardsupport_init(central_data_get_pointer_to_struct());
	central_data_init();
	
	run_all_tests();
	
	return 0;
}
