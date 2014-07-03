/*
 * radar_driver.h
 *
 * Created: 19/05/2012 00:29:41
 *  Author: sfx
 */ 


#ifndef RADAR_DRIVER_H_
#define RADAR_DRIVER_H_

#include "compiler.h"
#include "gpio.h"

#define RADAR_POWER1_PIN AVR32_PIN_PC17
#define RADAR_POWER2_PIN AVR32_PIN_PC18

void radar_driver_init(void);

void radar_driver_switch_power(int supply1, int supply2);


#endif

