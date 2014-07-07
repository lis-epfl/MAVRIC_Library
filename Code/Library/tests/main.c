#include <stdio.h>

#include "all_tests.h"
#include "delay.h"

#include "central_data.h"
#include "boardsupport.h"

int32_t main(int32_t argc, char **argv)
{
	
	boardsupport_init(central_data_get_pointer_to_struct());
	central_data_init();
	
	run_all_tests();

}
