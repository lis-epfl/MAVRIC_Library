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

#define I2C_DEVICES 2
#define I2C_SCHEDULE_SLOTS 10

#define I2C_READ 0
#define I2C_WRITE 1
#define I2C_WRITE1_THEN_READ 2

typedef struct {
	unsigned char	slave_address; // I2C address of slave
	uint32_t		i2c_speed;     // speed of i2c bus clock in kHz/kbps. Normally 100-400
	char			direction;
	char			write_then_read_preamble;
	unsigned char*  data;
	unsigned        data_size;
	unsigned		data_index;
	bool			transfer_in_progress;
} i2c_packet_t;


typedef void (i2c_callback_t)(i2c_packet_t* data);



int init_i2c(unsigned char i2c_device);

char i2c_reset(unsigned char i2c_device);




#endif /* I2C_DRIVER_H_ */