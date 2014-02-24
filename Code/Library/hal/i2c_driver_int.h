/*
 * i2c_driver.h
 *
 * Created: 14/05/2012 14:06:33
 *  Author: sfx
 */ 

#ifndef I2C_DRIVER_INT_H_
#define I2C_DRIVER_INT_H_


#include "twim.h"
#include "compiler.h"
#include "dma_channel_config.h"
#include "scheduler.h"

#define I2C_DEVICES 2
#define I2C_SCHEDULE_SLOTS 10

#define I2C_READ 0
#define I2C_WRITE 1
#define I2C_WRITE1_THEN_READ 2

typedef struct {
	uint8_t			slave_address; // I2C address of slave
	uint32_t		i2c_speed;     // speed of i2c bus clock in kHz/kbps. Normally 100-400
	char			direction;
	char			write_then_read_preamble;
	uint8_t			*data;
	uint16_t        data_size;
	uint16_t		data_index;
	bool			transfer_in_progress;
	task_handle_t	*event_handler;
} i2c_packet_t;



int init_i2c(unsigned char i2c_device);

int i2c_append_transfer(i2c_packet_t *request);
int i2c_append_read_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);
int i2c_append_write_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);
int i2c_append_register_read_transfer(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint16_t size, task_handle_t *event_handler);

int i2c_clear_queue(void);

bool i2c_is_ready(void);
int i2c_get_queued_requests(void);

#endif /* I2C_DRIVER_H_ */