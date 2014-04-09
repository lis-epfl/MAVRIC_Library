/*
 * time_keeper.h
 *
 * Created: 08/06/2012 17:25:26
 *  Author: sfx
 */ 


#ifndef TIME_KEEPER_H_
#define TIME_KEEPER_H_

#include "compiler.h"
#include "ast.h"

#define TK_AST_FREQUENCY 1000000   // timer ticks per second (32 bit timer, >1h time-out at 1MHz, >years at 1kHz. We'll go for precision here...)
#define AST_PRESCALER_SETTING 5    // log(SOURCE_CLOCK/AST_FREQ)/log(2)-1   . when running from PBA (64Mhz), 5 (1Mhz), or 15 (~1khz, not precisely though).

void init_time_keeper(void);


double get_time(void);          // time in seconds since system start

uint32_t get_millis(void);     //milliseconds since system start
uint32_t get_micros(void);     // microseconds since system start. Will run over after an hour.

uint32_t get_time_ticks(void); //raw timer ticks

float ticks_to_seconds(uint32_t timer_ticks);


void delay_micros(int microseconds);

void delay_until(uint32_t until_time);

#endif /* TIME_KEEPER_H_ */