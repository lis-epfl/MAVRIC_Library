#ifndef ALL_TESTS_H
#define ALL_TESTS_H

#include "test_maths.h"
#include "compiler.h"
static inline bool run_all_tests() {
	bool result=true;
	result= result && (run_math_tests());
	
	result=result && run_matrix_tests();
	
	return result;
}

#endif