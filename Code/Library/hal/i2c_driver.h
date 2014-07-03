/*
 * i2c_driver.h
 *
 * Created: 14/05/2012 14:06:33
 *  Author: sfx
 */ 

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_


#include "twim.h"
#include "compiler.h"
#include "dma_channel_config.h"

#define I2C_DEVICES 2
#define I2C_SCHEDULE_SLOTS 10

#define I2C_READ 0
#define I2C_WRITE 1
#define I2C_WRITE1_THEN_READ 2

typedef struct {
	unsigned char	slave_address; // I2C address of slave
	uint16_t		i2c_speed;     // speed of i2c bus clock in kHz/kbps. Normally 100-400
	char direction;
	char write_then_read_preamble;
	unsigned char*  write_data;
	unsigned        write_count;
	unsigned char*  read_data;
	unsigned        read_count;
} i2c_packet_conf;

typedef void (i2c_callback_t)(i2c_packet_conf* data);

typedef struct  {
	i2c_packet_conf config;
	unsigned char   schedule_slot;       // the assigned slot in the schedule  - READ ONLY!
	int             repetition_rate_ms;  // schedule repetition rate in milliseconds. A value of 0 means no repetition (one-shot)
	unsigned char   trigger_next_event;  // number of event that should be scheduled immediately after the end of this one (repetition for next event should be 0)
	char     active;                     // indicates if event should be scheduled or not. Will be set to false after a one-shot event 0:false, 1:true
	char     transfer_in_progress;        // flag indicates if this event is currently being processed. 0: false, 1: true, -1: uninitialised
	i2c_callback_t* callback;		     // callback function to be called when transfer finished.
} i2c_schedule_event ;



int i2c_driver_init(unsigned char i2c_device);

char i2c_driver_reset(unsigned char i2c_device);


char i2c_driver_add_request(unsigned char i2c_device, i2c_schedule_event* new_event);
char i2c_driver_change_request(unsigned char i2c_device, i2c_schedule_event* new_event);

char i2c_driver_enable_request(unsigned char i2c_device, unsigned char schedule_slot);
char i2c_driver_pause_request(unsigned char i2c_device, unsigned char schedule_slot);  // if the slot is currently processing, this blocks until it's finished
char i2c_driver_remove_request(unsigned char i2c_device, unsigned char schedule_slot);  // if the slot is currently processing, this blocks until it's finished
char i2c_driver_trigger_request(unsigned char i2c_device, unsigned char schedule_slot);


#endif /* I2C_DRIVER_H_ */