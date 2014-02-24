#ifndef ALL_TESTS_H
#define ALL_TESTS_H

#include "test_maths.h"
#include "test_small_matrix.h"
#include "test_quick_trig.h"

#include "compiler.h"
static inline bool run_all_tests() {
	bool result=true;
	result=result &&  run_math_tests();
	
	result=result && run_matrix_tests();

	result=result && run_quick_trig_tests();

	profile_quick_trig();
	
	return result;
}

#endif