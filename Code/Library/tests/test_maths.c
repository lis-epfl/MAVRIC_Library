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
 * \file test_maths.c
 * 
 * to Test math functions
 */
 
 
#include "test_maths.h"

#include "maths.h"
#include "print_util.h"

UQuat_t quat1= {.s=0.0, .v={0.0, 0.0, 0.0} };
UQuat_t quat2, quat3, quat4; 

float v[3];

float v2[3] = {-0.3, 0.44, 0.55};

bool run_math_tests() {
	
	bool result=true;
	
	v[0]=1.0; v[1]=0.0; v[2]=0.0;
	print_util_dbg_print_vector(v, 1);
	print_util_dbg_print_vector(v, 3);
	print_util_dbg_print("\n");
	print_util_dbg_print_vector(v2, 3);

	print_util_dbg_print_quaternion(&quat1, 3);	print_util_dbg_print(" (quat1)\n");
	
	quat1 = quaternions_create_from_vector(v);
	print_util_dbg_print_quaternion(&quat1, 3);    print_util_dbg_print(" (quat1 from v)\n");
	
	quat2 = quaternions_create_from_vector(v2);
	print_util_dbg_print_quaternion(&quat2, 3);    print_util_dbg_print(" (quat2 from v2)\n");
	
	quat2 = quaternions_normalise(quat2);
	print_util_dbg_print_quaternion(&quat2, 3);    print_util_dbg_print(" (quat2 normalised)\n");
	
	quat3 = quaternions_global_to_local(quat2, quat1);
	print_util_dbg_print_quaternion(&quat3, 4);    print_util_dbg_print(" (quat3 global to local)\n");
	
	quat4 = quaternions_local_to_global(quat2, quat3);
	print_util_dbg_print_quaternion(&quat4, 4);    print_util_dbg_print(" (quat4 local to global)\n");
	
	print_util_dbg_putfloat(maths_fast_sqrt(2), 6);   print_util_dbg_print(" sqrt(2)\n");
	print_util_dbg_putfloat(maths_fast_sqrt(4), 6);   print_util_dbg_print(" sqrt(4)\n");
	print_util_dbg_putfloat(maths_fast_sqrt(-4), 6);   print_util_dbg_print(" sqrt(-4)\n");
	return result;
}