/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file radar_driver.h
 * 
 * The radar driver
 */


#ifndef RADAR_DRIVER_H_
#define RADAR_DRIVER_H_

#include "compiler.h"
#include "gpio.h"

#define RADAR_POWER1_PIN AVR32_PIN_PC17		///< Define the microcontroller pin mapped with the radar power pin1
#define RADAR_POWER2_PIN AVR32_PIN_PC18		///< Define the microcontroller pin mapped with the radar power pin2

/**
 * \brief Initialize the radar driver
 *\
void radar_driver_init(void);

/**
 * \brief Switch the radar power to high or low
 *
 * \param supply1 set power pin1 to a high or low level
 * \param supply2 set power pin2 to a high or low level
 *\
void radar_driver_switch_power(int32_t supply1, int32_t supply2);


#endif

