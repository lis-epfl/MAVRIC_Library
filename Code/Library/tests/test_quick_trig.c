/*
 * test_quick_trig.c
 *
 * Created: 20/02/2014 15:08:08
 *  Author: sfx
 */ 
#include "test_quick_trig.h"
#include "quick_trig.h"


bool run_quick_trig_tests() {
	float input=0.0f;
	float output;
	bool test_result;
	float error=0.001;
	
	for (input=0.0; input<2*PI; input+=0.3) {
		TEST_ASSERT(sin, (f_abs(quick_sin(input)-sin(input)) < error), test_result);
		TEST_ASSERT(cos, (f_abs(quick_cos(input)-cos(input)) < error), test_result);
	}
	return test_result;
}

void profile_quick_trig() {
	uint32_t ex_time=0;
	float input=0.1234;
	static volatile float output;
	input=0.1234;
	//baseline
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=input;input=output+0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=input;input=output*1.123456f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=input;input=output/1.123456f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");

	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_sin(input);input=output+0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n");
	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output = sin(input); input = output +0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");

	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_cos(input);input=output+0.1f;  );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n");
	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output = cos(input); input = output +0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");

	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_acos(input);input=output*0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n");
	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output = acos(input); input = output *0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");

	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=quick_atan(input);input=output+0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n");
	input=0.1234;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output = atan(input); input = output +0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");
	
	input=12345.678;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=fast_sqrt(input);input=output+0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n");
	input=12345.678;
	dbg_print("start value in=");dbg_putfloat(input, 10);dbg_print("\n");
	PROFILE_100X(ex_time, output=      sqrt(input);input=output+0.1f; );
	dbg_print("in=");dbg_putfloat(input, 10); dbg_print("\tout=");dbg_putfloat(output, 10);dbg_print("\n\n");

	
}
