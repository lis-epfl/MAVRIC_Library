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
 * \file i2c_driver.h
 * 
 * The i2c driver
 */
 
 
#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_


#include "twim.h"
#include "compiler.h"
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
	unsigned char	slave_address;	///< I2C address of slave
	uint16_t		i2c_speed;     	///< speed of i2c bus clock in kHz/kbps. Normally 100-400
	char direction;					///< i2c direction transfer
	char write_then_read_preamble;	///< i2c mode
	unsigned char*  write_data;		///< i2c write data buffer
	unsigned        write_count;	///< i2c write counter
	unsigned char*  read_data;		///< i2c read data buffer
	unsigned        read_count;		///< i2c read counter
} i2c_packet_conf;

typedef void (i2c_callback_t)(i2c_packet_conf* data);

/**
 * \brief i2c event scheduler packet
 */
typedef struct  
{
	i2c_packet_conf config;
	unsigned char   schedule_slot;       ///< the assigned slot in the schedule  - READ ONLY!
	int             repetition_rate_ms;  ///< schedule repetition rate in milliseconds. A value of 0 means no repetition (one-shot)
	unsigned char   trigger_next_event;  ///< number of event that should be scheduled immediately after the end of this one (repetition for next event should be 0)
	char     active;                     ///< indicates if event should be scheduled or not. Will be set to false after a one-shot event 0:false, 1:true
	char     transfer_in_progress;       ///< flag indicates if this event is currently being processed. 0: false, 1: true, -1: uninitialised
	i2c_callback_t* callback;		     ///< callback function to be called when transfer finished.
} i2c_schedule_event ;

/**
 * \brief Initialize the i2c driver
 *
 * \param i2c_device the i2c device number
 *
 * \return the initializing status
 */
int i2c_driver_init(unsigned char i2c_device);

/**
 * \brief Reset the i2c driver
 *
 * \param i2c_device the i2c device number
 *
 * \return the error status
 */
char i2c_driver_reset(unsigned char i2c_device);

/**
 * \brief Add a request to the i2c device scheduler
 *
 * \param i2c_device the i2c device
 * \param new_event a pointer to the event scheduler
 *
 * \return the error status
 */
char i2c_driver_add_request(unsigned char i2c_device, i2c_schedule_event* new_event);

/**
 * \brief Change the i2c request
 *
 * \param i2c_device the i2c device
 * \param new_event a pointer to the event scheduler
 *
 * \return the error status
 */
char i2c_driver_change_request(unsigned char i2c_device, i2c_schedule_event* new_event);

/**
 * \brief Enable an i2c request
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
char i2c_driver_enable_request(unsigned char i2c_device, unsigned char schedule_slot);

/**
 * \brief Pause the i2c request
 * if the slot is currently processing, this blocks until it's finished
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
char i2c_driver_pause_request(unsigned char i2c_device, unsigned char schedule_slot);

/**
 * \brief Remove an i2c request
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
char i2c_driver_remove_request(unsigned char i2c_device, unsigned char schedule_slot);  // if the slot is currently processing, this blocks until it's finished

/**
 * \brief Trigger an i2c request
 *
 * \param i2c_device the i2c device
 * \param schedule_slot the i2c slot of the scheduler
 *
 * \return the error status
 */
char i2c_driver_trigger_request(unsigned char i2c_device, unsigned char schedule_slot);
#endif /* I2C_DRIVER_H_ */