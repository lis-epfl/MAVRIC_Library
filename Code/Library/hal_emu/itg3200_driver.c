/*
 * itg3200.c
 *
 * Created: 18/05/2012 17:57:46
 *  Author: sfx
 */ 


#include "itg3200.h"

//#include "i2c_driver_int.h"
#include "print_util.h"
//#include "twim.h"

static volatile gyroscope_t gyro_outputs;

//static  i2c_schedule_event_t gyro_event;

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

void itg3200_init(void) {

}

void itg3200_reconfigure_gyro(void) {
	
}

void itg3200_init_slow(void) {

}

gyroscope_t* itg3200_get_gyro_data(void) {
	
	return &gyro_outputs;
}

gyroscope_t* itg3200_get_data_slow(void) {
	
	
	
	return &gyro_outputs;
}