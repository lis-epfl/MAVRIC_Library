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
 * \file i2c_driver_int.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for i2c with interruptions
 *
 ******************************************************************************/

#ifndef I2C_DRIVER_INT_H_
#define I2C_DRIVER_INT_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "twim.h"
#include <stdint.h>
#include "dma_channel_config.h"
#include "scheduler.h"

#define I2C_DEVICES 2				///< Define the I2C maximum number of devices
#define I2C_SCHEDULE_SLOTS 10		///< Define the max number of slots for the scheduling of the i2c

#define I2C_READ 0					///< Define the I2C mode to Read
#define I2C_WRITE 1					///< Define the I2C mode to Write
#define I2C_WRITE1_THEN_READ 2		///< Define the I2C mode to Write followed by read

/**
 * \brief Enumerate the 2 possible I2C
 */
enum AVAILABLE_I2CS
{
	I2C0, 
	I2C1
};

/**
 * \brief structure to embed the i2c's data
*/
typedef struct 
{
	uint8_t			slave_address;				///< I2C address of slave
	uint32_t		i2c_speed;					///< speed of i2c bus clock in kHz/kbps. Normally 100-400
	int8_t 			direction;					///< direction of the bus
	int8_t 			write_then_read_preamble;	///< preamble in "write follow by read" mode
	uint8_t			*data;						///< pointer to a data buffer of uint8_t
	uint16_t        data_size;					///< size of the data buffer
	uint16_t		data_index;					///< index of the data buffer
	bool			transfer_in_progress;		///< define whether the transfer is in progress
	task_handle_t	*event_handler;				///< pointer to a task handler buffer
} i2c_packet_t;

/**
 * \brief Initialize the I2C communication
 *
 * \param	i2c_device		select which device to initialize
 * \param	twi_opt			i2c	driver configurations
*/
void i2c_driver_init(uint8_t  i2c_device, twim_options_t twi_opt);

/**
 * \brief Append the i2c transfer
 *
 * \param	request			pointer to the i2c packet
 *
 * \return nothing implemented yet
*/
int32_t i2c_append_transfer(i2c_packet_t *request);

/**
 * \brief Append the i2c read transfer
 *
 * \param	slave_address	Address of the Slave device which we were reading to
 * \param	data			Pointer the data buffer
 * \param	size			Size of the data buffer
 * \param	event_handler	Pointer to a task handler
 *
 * \return nothing implemented yet
*/
int32_t i2c_append_read_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

/**
 * \brief append the i2c write transfer
 *
 * \param	slave_address	Address of the Slave device which we were reading to
 * \param	data			Pointer the data buffer
 * \param	size			Size of the data buffer
 * \param	event_handler	Pointer to a task handler
 *
 * \return nothing implemented yet
*/
int32_t i2c_append_write_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

/**
 * \brief 
 *
  * \param	slave_address		Address of the Slave device which we were reading to
  * \param	register_address	Address of a register on the slave
  * \param	data				Pointer the data buffer
  * \param	size				Size of the data buffer
  * \param	event_handler		Pointer to a task handler
  *
 * \return nothing implemented yet
*/
int32_t i2c_append_register_read_transfer(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

/**
 * \brief Clear the queue of the i2c
 *
 * \return nothing implemented yet
*/
int32_t i2c_clear_queue(void);

/**
 * \brief Return whether the i2c is ready to use
 *
 * \return nothing implemented yet
*/
bool i2c_is_ready(void);

/**
 * \brief Get the request which is queued
 *
 * \return nothing implemented yet
*/
int32_t i2c_get_queued_requests(void);

#ifdef __cplusplus
	}
#endif

#endif /* I2C_DRIVER_H_ */