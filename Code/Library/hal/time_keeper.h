/**
 * \page The MAV'RIC license
 * 
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file time_keeper.h
 *
 * This file is used to interact with the clock of the microcontroller
 */


#ifndef TIME_KEEPER_H_
#define TIME_KEEPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "ast.h"

#define TK_AST_FREQUENCY 1000000					///< Timer ticks per second (32 bit timer, >1h time-out at 1MHz, >years at 1kHz. We'll go for precision here...)
#define AST_PRESCALER_SETTING 5						///< Log(SOURCE_CLOCK/AST_FREQ)/log(2)-1 when running from PBA (64Mhz), 5 (1Mhz), or 15 (~1khz, not precisely though).

/** 
 * \brief	This function initialize the clock of the microcontroller
 */
void time_keeper_init(void);

/** 
 * \brief	This function returns the time in seconds since system start
 * 
 * \return	The time in seconds since system start
 */
double time_keeper_get_time(void);

/**
 * \brief	This function returns the time in milliseconds since system start
 *
 * \return The time in milliseconds since system start
 */
uint32_t time_keeper_get_millis(void);

/**
 * \brief	This function returns the time in microseconds since system start. 
 *
 * \warning	Will run over after an hour.
 *
 * \return The time in microseconds since system start
 */
uint32_t time_keeper_get_micros(void);

/**
 * \brief	raw timer ticks
 *
 * \return	The raw timer ticks
 */
uint32_t time_keeper_get_time_ticks(void);

/**
 * \brief	Transforms the timer ticks into seconds
 *
 * \param	timer_ticks		The timer ticks
 *
 * \return	The time in seconds
 */
float time_keeper_ticks_to_seconds(uint32_t timer_ticks);

/**
 * \brief	Functions that runs for the parameters input microseconds before returning
 *
 * \param	microseconds		The number of microseconds to wait
 */
void time_keeper_delay_micros(int32_t microseconds);

/**
 * \brief	Wait until time pass the parameter input
 *
 * \param	until_time		The time until which the function will run
 */
void time_keeper_delay_until(uint32_t until_time);

#ifdef __cplusplus
}
#endif

#endif /* TIME_KEEPER_H_ */