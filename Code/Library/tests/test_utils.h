/*
 * test_utils.h
 *
 * Created: 20/02/2014 15:16:44
 *  Author: sfx
 */ 


#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include "print_util.h"
#include "time_keeper.h"
#include "delay.h"

#define xstr(a) str(a)
#define str(a) #a

#define TEST_ASSERT(TEST_NAME, CONDITION, RESULT_OUT) {\
	if (CONDITION) {\
		dbg_print(str(TEST_NAME));\
		dbg_print(": passed\n");\
	} else {\
		dbg_print(str(TEST_NAME));\
		dbg_print(": FAILED!\n");\
		RESULT_OUT=false;\
	}\
}

#define PROFILE(TIME_RESULT, TESTFUNC) {\
	get_debug_stream()->flush(get_debug_stream()->data);\
	delay_ms(10);\
	uint32_t __profile_time_start=time_keeper_get_time_ticks();\
	TESTFUNC;\
	TIME_RESULT=time_keeper_get_time_ticks()-__profile_time_start);\
	dbg_print("Profiling: ");\
	dbg_print(str(TESTFUNC)"\t");\
	dbg_print_num(TIME_RESULT, 10);\
	dbg_print(" microseconds\n");\
	get_debug_stream()->flush(get_debug_stream()->data);\
	delay_ms(10);\
}


#define PROFILE_10X(TIME_RESULT, TESTFUNC) {\
	get_debug_stream()->flush(get_debug_stream()->data);\
	delay_ms(10);\
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
	dbg_print("Profiling (10x): ");\
	dbg_print(#TESTFUNC "\t");\
	dbg_print_num(TIME_RESULT, 10);\
	dbg_print(" microseconds\n");\
	get_debug_stream()->flush(get_debug_stream()->data);\
	delay_ms(10);\
}

#define PROFILE_100X(TIME_RESULT, TESTFUNC) {\
	get_debug_stream()->flush(get_debug_stream()->data);\
	delay_ms(10);\
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
	dbg_print("Profiling (100x): ");\
	dbg_print(#TESTFUNC "\t");\
	dbg_print_num(TIME_RESULT, 10);\
	dbg_print(" microseconds\n");\
	get_debug_stream()->flush(get_debug_stream()->data);\
	delay_ms(10);\
}



#endif /* TEST_UTILS_H_ */