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
* \file bmp085.h
*
* This file is the driver for the barometer module: BMP085
*/


#ifndef BMP085_H_
#define BMP085_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "scheduler.h"
#include <stdint.h>
#include <stdbool.h>


/**
 * \brief pressure_sensor_state_t can get three different state: Idle, get Temperature or get Pressure
*/
typedef enum pressure_sensor_state_t
{
	IDLE,				///< Idle state
	GET_TEMP,			///< Getting temperature state
	GET_PRESSURE		///< Getting pressure state
} pressure_sensor_state_t;


/**
 * \brief structure containing all the barometer's data
*/
typedef struct pressure_data_t
{
	uint8_t raw_pressure[3];		///< Raw pressure contained in 3 uint8_t
	uint8_t raw_temperature[2];		///< Raw temperature contained in 2 uint8_t
	float pressure;					///< Measured pressure as the concatenation of the 3 uint8_t raw_pressure
	float temperature;				///< Measured temperature as the concatenation of the 2 uint8_t raw_temperature
	float last_altitudes[3];		///< Array to store previous value of the altitude for low pass filtering the output
	float altitude;					///< Measured altitude as the median filter of the 3 last_altitudes
	float altitude_offset;			///< Offset of the barometer sensor for matching GPS altitude value
	float vario_vz;					///< Vario altitude speed
	uint32_t last_update;			///< Time of the last update of the barometer
	uint32_t last_state_update;		///< Time of the last state update
	pressure_sensor_state_t state;	///< State of the barometer sensor (IDLE, GET_TEMP, GET_PRESSURE)
	float dt;						///< Time step for the derivative
	
	const mavlink_stream_t* mavlink_stream;			///< The pointer to the mavlink stream structure
} pressure_data_t;


/**
 * \brief Initialize the barometer sensor
*/
void bmp085_init(pressure_data_t *baro, const mavlink_stream_t * mavlink_stream);


/**
 * \brief Initialize the barometer sensor in slow mode
*/
void bmp085_init_slow(void);


/**
 * \brief	Initialization of the pos_est->barometer offset
 *
 * \param	pos_est			The pointer to the position estimation structure
 * \param	pos_est->barometer		The pointer to the pos_est->barometer structure
 *
 * \return	void
 */
void bmp085_reset_origin_altitude(pressure_data_t* baro, float origin_altitude);


/**
 * \brief Get the pressure data n slow mode
 *
 * \param offset Offset to add to the pressure measured by the barometer
 *
 * \return a pointer to the pressure data structure
*/
void bmp085_update(pressure_data_t *baro);


/**
 * \brief	Task to send the mavlink scaled pressure message
 * 
 * \param	pressure	The pointer to the pressure structure
 *
 * \return	The status of execution of the task
 */
task_return_t bmp085_send_pressure(pressure_data_t* baro);


#ifdef __cplusplus
}
#endif

#endif /* BMP085_H_ */