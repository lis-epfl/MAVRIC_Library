#include "test_maths.h"

#include "maths.h"
#include "print_util.h";

UQuat_t quat1= {.s=0.0, .v={0.0, 0.0, 0.0} };
UQuat_t quat2, quat3, quat4; 

float v[3];

float v2[3] = {-0.3, 0.44, 0.55};

int run_math_tests() {
	
	bool result=true;
	
	v[0]=1.0; v[1]=0.0; v[2]=0.0;
	dbg_print_vector(v, 1);
	dbg_print_vector(v, 3);
	dbg_print("\n");
	dbg_print_vector(v2, 3);

	dbg_print_quaternion(&quat1, 3);	dbg_print(" (quat1)\n");
	quat1=quat_from_vector(v);
	dbg_print_quaternion(&quat1, 3);    dbg_print(" (quat1 from v)\n");
	quat2=quat_from_vector(v2);
	dbg_print_quaternion(&quat2, 3);    dbg_print(" (quat2 from v2)\n");
	quat2=quat_normalise(quat2);
	dbg_print_quaternion(&quat2, 3);    dbg_print(" (quat2 normalised)\n");
	
	quat3=quat_global_to_local(quat2, quat1);
	dbg_print_quaternion(&quat3, 4);    dbg_print(" (quat3 global to local)\n");
	
	quat4=quat_local_to_global(quat2, quat3);
	dbg_print_quaternion(&quat4, 4);    dbg_print(" (quat4 local to global)\n");
	
	dbg_putfloat(fast_sqrt(2), 6);   dbg_print(" sqrt(2)\n");
	dbg_putfloat(fast_sqrt(4), 6);   dbg_print(" sqrt(4)\n");
	dbg_putfloat(fast_sqrt(-4), 6);   dbg_print(" sqrt(-4)\n");
	
}