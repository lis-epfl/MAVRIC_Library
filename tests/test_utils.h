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


#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include "print_util.h"
#include "time_keeper.h"
#include "delay.h"

#define xstr(a) str(a)		///< Define strings
#define str(a) #a 			///< Define strings

#define TEST_ASSERT(TEST_NAME, CONDITION, RESULT_OUT) {\
	if (CONDITION) {\
		print_util_dbg_print(str(TEST_NAME));\
		print_util_dbg_print(": passed\n");\
	} else {\
		print_util_dbg_print(str(TEST_NAME));\
		print_util_dbg_print(": FAILED!\n");\
		RESULT_OUT=false;\
	}\
}

#define PROFILE(TIME_RESULT, TESTFUNC) {\
	print_util_get_debug_stream()->flush(print_util_get_debug_stream()->data);\
	time_keeper_delay_ms(10);\
	uint32_t __profile_time_start=time_keeper_get_time_ticks();\
	TESTFUNC;\
	TIME_RESULT=time_keeper_get_time_ticks()-__profile_time_start);\
	print_util_dbg_print("Profiling: ");\
	print_util_dbg_print(str(TESTFUNC)"\t");\
	print_util_dbg_print_num(TIME_RESULT, 10);\
	print_util_dbg_print(" microseconds\n");\
	print_util_get_debug_stream()->flush(print_util_get_debug_stream()->data);\
	time_keeper_delay_ms(10);\
}


#define PROFILE_10X(TIME_RESULT, TESTFUNC) {\
	print_util_get_debug_stream()->flush(print_util_get_debug_stream()->data);\
	time_keeper_delay_ms(10);\
	uint32_t __profile_time_start=time_keeper_get_time_ticks();\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TESTFUNC;\
	TIME_RESULT=time_keeper_get_time_ticks()-__profile_time_start;\
	print_util_dbg_print("Profiling (10x): ");\
	print_util_dbg_print(#TESTFUNC "\t");\
	print_util_dbg_print_num(TIME_RESULT, 10);\
	print_util_dbg_print(" microseconds\n");\
	print_util_get_debug_stream()->flush(print_util_get_debug_stream()->data);\
	time_keeper_delay_ms(10);\
}

#define PROFILE_100X(TIME_RESULT, TESTFUNC) {\
	print_util_get_debug_stream()->flush(print_util_get_debug_stream()->data);\
	time_keeper_delay_ms(10);\
	uint32_t __profile_time_start=time_keeper_get_time_ticks();\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;	TESTFUNC;\
	TIME_RESULT=time_keeper_get_time_ticks()-__profile_time_start;\
	print_util_dbg_print("Profiling (100x): ");\
	print_util_dbg_print(#TESTFUNC "\t");\
	print_util_dbg_print_num(TIME_RESULT, 10);\
	print_util_dbg_print(" microseconds\n");\
	print_util_get_debug_stream()->flush(print_util_get_debug_stream()->data);\
	time_keeper_delay_ms(10);\
}



#endif /* TEST_UTILS_H_ */