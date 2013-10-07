#ifndef ALL_TESTS_H
#define ALL_TESTS_H

#include "test_maths.h"

static bool inline run_all_tests() {
	bool result= (run_math_tests());
	
	
	return result;
}

#endif