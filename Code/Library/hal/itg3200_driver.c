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

static  i2c_schedule_event gyro_event;

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

void init_itg3200(void) {
	default_configuration.conf_start_reg_address=CONFIG_REG_ADDRESS;

	default_configuration.sample_div=4; //output frequency after filtering: 1khz/8khz /(sample_div +1)
	default_configuration.DLPF=DLPF_42HZ;
	default_configuration.interrupts=0;

	gyro_event.callback=0;
	gyro_event.repetition_rate_ms=5;
	gyro_event.trigger_next_event=-1;

	gyro_event.config.slave_address=ITG3200_SLAVE_ADDRESS;
	gyro_event.config.direction=I2C_WRITE;
	gyro_event.config.read_data=&gyro_outputs;
	gyro_event.config.read_count=8;
	gyro_event.config.write_data=&default_configuration;
	gyro_event.config.write_count=4;
	gyro_event.config.i2c_speed=100000;
	
	i2c_add_request(0, &gyro_event);
	i2c_trigger_request(0, gyro_event.schedule_slot);
	
	
	gyro_event.config.direction=I2C_WRITE1_THEN_READ;
	gyro_event.config.write_then_read_preamble=SENSOR_REG_ADDRESS;
	gyro_event.config.write_data=&read_preamble;
	gyro_event.config.write_count=1;
	
	gyro_event.config.read_data=&gyro_outputs;
	gyro_event.config.read_count=8;
	i2c_change_request(0, &gyro_event);
	i2c_trigger_request(0, gyro_event.schedule_slot);
	/**/
}

void reconfigure_gyro(void) {
	i2c_trigger_request(0, gyro_event.schedule_slot);
}

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

gyro_data* get_gyro_data(void) {
	i2c_trigger_request(0, gyro_event.schedule_slot);
	return &gyro_outputs;
}

gyro_data* get_gyro_data_slow(void) {
	gyro_event.config.write_then_read_preamble=SENSOR_REG_ADDRESS;
	twim_write(&AVR32_TWIM0, (uint8_t*) &gyro_event.config.write_then_read_preamble, 1, ITG3200_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&gyro_outputs, 8, ITG3200_SLAVE_ADDRESS, false);
	
	
	return &gyro_outputs;
}