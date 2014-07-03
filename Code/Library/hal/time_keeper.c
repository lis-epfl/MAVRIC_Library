 /* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file time_keeper.c
 *
 * This file is used to interact with the clock of the microcontroller
 */


#include "time_keeper.h"

void time_keeper_init()
{
	ast_init_counter(&AVR32_AST, AST_OSC_PB, AST_PRESCALER_SETTING, 0);
	ast_enable(&AVR32_AST);
}

double time_keeper_get_time()
{
	// time in seconds since system start
	return time_keeper_ticks_to_seconds(time_keeper_get_time_ticks());
}

uint32_t time_keeper_get_millis()
{
	//milliseconds since system start
	return time_keeper_get_time_ticks()/1000; /// (TK_AST_FREQUENCY/1000);
}

uint32_t time_keeper_get_micros()
{
	// microseconds since system start. Will run over after an hour.
	return time_keeper_get_time_ticks()* (1000000/TK_AST_FREQUENCY);
}

uint32_t time_keeper_get_time_ticks()
{
	//raw timer ticks
	return ast_get_counter_value(&AVR32_AST);
}

float time_keeper_ticks_to_seconds(uint32_t timer_ticks)
{
	return ((double)timer_ticks/(double)TK_AST_FREQUENCY);
}

void time_keeper_delay_micros(int microseconds)
{
	uint32_t now=time_keeper_get_micros();
	while (time_keeper_get_micros()<now+microseconds);
}

void time_keeper_delay_until(uint32_t until_time)
{
	while (time_keeper_get_micros()<until_time);
}
