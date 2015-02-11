/*
 * time_keeper.c
 *
 * Created: 08/06/2012 17:48:13
 *  Author: Felix Schill
 */ 

#include <sys/time.h>
#include "time_keeper.h"

#define TK_AST_FREQUENCY 1000000					///< Timer ticks per second (32 bit timer, >1h time-out at 1MHz, >years at 1kHz. We'll go for precision here...)
#define AST_PRESCALER_SETTING 5						///< Log(SOURCE_CLOCK/AST_FREQ)/log(2)-1 when running from PBA (64Mhz), 5 (1Mhz), or 15 (~1khz, not precisely though).


void time_keeper_init() 
{
	;
}


uint32_t time_keeper_get_time_ticks()
{ 	
	//raw timer ticks
	struct timeval tv;
	
	gettimeofday(&tv,0);
	return tv.tv_sec*1000000+tv.tv_usec;
}


double time_keeper_get_time()
{
	// time in seconds since system start
	return time_keeper_ticks_to_seconds(time_keeper_get_time_ticks());
}


uint32_t time_keeper_get_millis()
{
	//milliseconds since system start
	return time_keeper_get_time_ticks() / 1000; /// (TK_AST_FREQUENCY / 1000);
}


uint32_t time_keeper_get_micros()
{
	// microseconds since system start. Will run over after an hour.
	return time_keeper_get_time_ticks() * (1000000 / TK_AST_FREQUENCY);
}


float time_keeper_ticks_to_seconds(uint32_t timer_ticks)
{
	return ((double)timer_ticks / (double)TK_AST_FREQUENCY);
}


void time_keeper_delay_micros(int32_t microseconds)
{
	uint32_t now = time_keeper_get_micros();
	while (time_keeper_get_micros() < now + microseconds);
}


void time_keeper_delay_until(uint32_t until_time)
{
	while (time_keeper_get_micros() < until_time);
}