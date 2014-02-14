#include <stdio.h>

#include "all_tests.h"
#include "delay.h"

#include "central_data.h"
#include "boardsupport.h"

int main(int argc, char **argv)
{
	
	initialise_board(get_central_data());
	initialise_central_data();
	
	run_all_tests();

}
