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
* \file i2c_driver_int.h
*
* This file is the driver for i2c with interruptions
*/


#ifndef I2C_DRIVER_INT_H_
#define I2C_DRIVER_INT_H_

#include "twim.h"
#include "compiler.h"
#include "dma_channel_config.h"
#include "scheduler.h"

#define I2C_DEVICES 2				///< Define the I2C maximum number of devices
#define I2C_SCHEDULE_SLOTS 10		///< Define the max number of slots for the scheduling of the i2c

#define I2C_READ 0					///< Define the I2C mode to Read
#define I2C_WRITE 1					///< Define the I2C mode to Write
#define I2C_WRITE1_THEN_READ 2		///< Define the I2C mode to Write followed by read

/**
 * \brief structure to embed the i2c's data
*/
typedef struct 
{
	uint8_t			slave_address;				///< I2C address of slave
	uint32_t		i2c_speed;					///< speed of i2c bus clock in kHz/kbps. Normally 100-400
	char			direction;					///< direction of the bus
	char			write_then_read_preamble;	///< preamble in "write follow by read" mode
	uint8_t			*data;						///< pointer to a data buffer of uint8_t
	uint16_t        data_size;					///< size of the data buffer
	uint16_t		data_index;					///< index of the data buffer
	bool			transfer_in_progress;		///< define whether the transfer is in progress
	task_handle_t	*event_handler;				///< pointer to a task handler buffer
} i2c_packet_t;

/**
 * \brief Initialize the I2C communication
 *
 * \param i2c_device select which device to initialize
 *
 * \return -1 if the device selected is not correct, -9 if the speed setting failed or STATUS_OK = 0 otherwise
*/
int init_i2c(unsigned char i2c_device);

/**
 * \brief Append the i2c transfer
 *
 * \param request pointer to the i2c packet
 *
 * \return nothing implemented yet
*/
int i2c_append_transfer(i2c_packet_t *request);

/**
 * \brief Append the i2c read transfer
 *
 * \param slave_address Address of the Slave device which we were reading to
 * \param data pointer the data buffer
 * \param size size of the data buffer
 * \param event_handler pointer to a task handler
 *
 * \return nothing implemented yet
*/
int i2c_append_read_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

/**
 * \brief append the i2c write transfer
 *
 * \param slave_address Address of the Slave device which we were reading to
 * \param data pointer the data buffer
 * \param size size of the data buffer
 * \param event_handler pointer to a task handler
 *
 * \return nothing implemented yet
*/
int i2c_append_write_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

/**
 * \brief 
 *
 * \param slave_address Address of the Slave device which we were reading to
 * \param register_address Address of a register on the slave
 * \param data pointer the data buffer
 * \param size size of the data buffer
 * \param event_handler pointer to a task handler
 *
 * \return nothing implemented yet
*/
int i2c_append_register_read_transfer(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

/**
 * \brief Clear the queue of the i2c
 *
 * \return nothing implemented yet
*/
int i2c_clear_queue(void);

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
int i2c_get_queued_requests(void);

#endif /* I2C_DRIVER_H_ */