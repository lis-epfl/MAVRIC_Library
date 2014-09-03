/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file i2c_driver.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief The i2c driver
 *
 ******************************************************************************/
 
 
#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_


#include "twim.h"
#include <stdint.h>
#include "dma_channel_config.h"

#define I2C_DEVICES 2				///< Define the max number of i2c devices
#define I2C_SCHEDULE_SLOTS 10		///< Define the max number of i2c scheduler's slots

#define I2C_READ 0					///< Define the i2c read flag
#define I2C_WRITE 1					///< Define the i2c write flag
#define I2C_WRITE1_THEN_READ 2		///< Define the i2c write followed by read flag

/**
 * \brief i2c configuration packet 
 */
typedef struct 
{
	uint8_t		slave_address;				///< I2C address of slave
	uint16_t	i2c_speed;     				///< speed of i2c bus clock in kHz/kbps. Normally 100-400
	int8_t 		direction;					///< i2c direction transfer
	int8_t 		write_then_read_preamble;	///< i2c mode
	uint8_t*  	write_data;					///< i2c write data buffer
	uint32_t    write_count;				///< i2c write counter
	uint8_t*  	read_data;					///< i2c read data buffer
	uint32_t    read_count;					///< i2c read counter
} i2c_packet_conf_t;

typedef void (i2c_callback_t)(i2c_packet_conf_t* data);

/**
 * \brief i2c event scheduler packet
 */
typedef struct  
{
	i2c_packet_conf_t config;
	uint8_t   		schedule_slot;       	///< the assigned slot in the schedule  - READ ONLY!
	int32_t   		repetition_rate_ms;  	///< schedule repetition rate in milliseconds. A value of 0 means no repetition (one-shot)
	uint8_t   		trigger_next_event;  	///< number of event that should be scheduled immediately after the end of this one (repetition for next event should be 0)
	int8_t    		active;              	///< indicates if event should be scheduled or not. Will be set to false after a one-shot event 0:false, 1:true
	int8_t    		transfer_in_progress;   ///< flag indicates if this event is currently being processed. 0: false, 1: true, -1: uninitialised
	i2c_callback_t* callback;		     	///< callback function to be called when transfer finished.
} i2c_schedule_event_t ;

/**
 * \brief Initialize the i2c driver
 *
 * \param i2c_device the i2c device number
 *
 * \return the initializing status
 */
int32_t i2c_driver_init(uint8_t i2c_device);

/**
 * \brief Reset the i2c driver
 *
 * \param i2c_device the i2c device number
 *
 * \return the error status
 */
int8_t i2c_driver_reset(uint8_t i2c_device);

/**
 * \brief Add a request to the i2c device scheduler
 *
 * \param i2c_device the i2c device
 * \param new_event a pointer to the event scheduler
 *
 * \return the error status
 */
int8_t i2c_driver_add_request(uint8_t i2c_device, i2c_schedule_event_t* new_event);

/**
 * \brief Change the i2c request
 *
 * \param i2c_device the i2c device
 * \param new_event a pointer to the event scheduler
 *
 * \return the error status
 */
int8_t i2c_driver_change_request(uint8_t i2c_device, i2c_schedule_event_t* new_event);

/**
 * \brief Enable an i2c request
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
int8_t i2c_driver_enable_request(uint8_t i2c_device, uint8_t schedule_slot);

/**
 * \brief Pause the i2c request
 * if the slot is currently processing, this blocks until it's finished
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
int8_t i2c_driver_pause_request(uint8_t i2c_device, uint8_t schedule_slot);

/**
 * \brief Remove an i2c request
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
int8_t i2c_driver_remove_request(uint8_t i2c_device, uint8_t schedule_slot);  // if the slot is currently processing, this blocks until it's finished

/**
 * \brief Trigger an i2c request
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
int8_t i2c_driver_trigger_request(uint8_t i2c_device, uint8_t schedule_slot);
#endif /* I2C_DRIVER_H_ */