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
 * \file all_tests.h
 * 
 * Test functions
 */
 
#ifndef ALL_TESTS_H
#define ALL_TESTS_H

#include "test_maths.h"
#include "test_small_matrix.h"
#include "test_quick_trig.h"

#include <stdint.h>

/**
 * \brief Run all the test
 *
 * \return the error status
 */
static inline bool run_all_tests() {
	bool result=true;
	result=result &&  run_math_tests();
	
	result=result && run_matrix_tests();

	result=result && run_quick_trig_tests();

	profile_quick_trig();
	
	return result;
}

#endif