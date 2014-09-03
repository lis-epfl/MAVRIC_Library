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
 * \file test_quick_trig.c
 * 
 * to Test simplyfied trigonometric functions
 */
 
 
#include "test_quick_trig.h"
#include "quick_trig.h"


bool run_quick_trig_tests(void) 
{
	float input=0.0f;
	bool test_result;
	float error=0.001;
	
	for (input=0.0; input<2*PI; input+=0.3) 
	{
		TEST_ASSERT(sin, (maths_f_abs(quick_trig_sin(input)-sin(input)) < error), test_result);
		TEST_ASSERT(cos, (maths_f_abs(quick_trig_cos(input)-cos(input)) < error), test_result);
	}
	return test_result;
}

void profile_quick_trig(void) 
{
	uint32_t ex_time=0;
	float input=0.1234;
	static volatile float output;
	input=0.1234;
	//baseline
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=input;input=output+0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=input;input=output*1.123456f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=input;input=output/1.123456f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");

	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_trig_sin(input);input=output+0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n");
	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output = sin(input); input = output +0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");

	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_trig_cos(input);input=output+0.1f;  );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n");
	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output = cos(input); input = output +0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");

	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_trig_acos(input);input=output*0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n");
	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output = acos(input); input = output *0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");

	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_trig_atan(input);input=output+0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n");
	input=0.1234;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output = atan(input); input = output +0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");
	
	input=12345.678;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=maths_fast_sqrt(input);input=output+0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n");
	input=12345.678;
	print_util_dbg_print("start value in=");print_util_dbg_putfloat(input, 10);print_util_dbg_print("\n");
	PROFILE_100X(ex_time, output=      sqrt(input);input=output+0.1f; );
	print_util_dbg_print("in=");print_util_dbg_putfloat(input, 10); print_util_dbg_print("\tout=");print_util_dbg_putfloat(output, 10);print_util_dbg_print("\n\n");
}
