/*
 * itg3200_driver.c
 *
 * Created: 18/05/2012 17:57:46
 *  Author: sfx
 */ 


#include "itg3200_driver.h"

#include "i2c_driver_int.h"
#include "print_util.h"
//#include "twim.h"

static volatile gyro_data gyro_outputs;


#define CONFIG_REG_ADDRESS 21
#define SENSOR_REG_ADDRESS 27

#define FULL_SCALE_MASK (_BV(3)|_BV(4))
#define DLPF_256HZ 0
#define DLPF_188HZ 1
#define DLPF_98HZ 2
#define DLPF_42HZ 3
#define DLPF_20HZ 4
#define DLPF_10HZ 5
#define DLPF_5HZ 6

typedef struct {
	uint8_t conf_start_reg_address;
	uint8_t sample_div;
	uint8_t DLPF;
	uint8_t interrupts;
	
} gyro_config;

gyro_config default_configuration;
uint8_t read_preamble=SENSOR_REG_ADDRESS;



void init_itg3200_slow(void) {
	static twim_options_t twi_opt= {
		.pba_hz=64000000, 
		.speed = 400000,
		.chip = ITG3200_SLAVE_ADDRESS, 
		.smbus=false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&default_configuration, 4, ITG3200_SLAVE_ADDRESS, false);
}


gyro_data* get_gyro_data_slow(void) {
	uint8_t write_then_read_preamble=SENSOR_REG_ADDRESS;
	twim_write(&AVR32_TWIM0, (uint8_t*) &write_then_read_preamble, 1, ITG3200_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&gyro_outputs, 8, ITG3200_SLAVE_ADDRESS, false);
	
	
	return &gyro_outputs;
}