/*
 * tasks.h
 *
 * Created: 16/09/2013 13:27:59
 *  Author: sfx
 */ 


#ifndef TASKS_H_
#define TASKS_H_

#include "scheduler.h"
#include "mavlink_actions.h"
#include "radar_module_driver.h"

typedef const struct {
	U8 var8;
	U16 var16;
	U32 var32;
} nvram_datas_t;

nvram_datas_t *ptr_nvram;

static const U8 write_data[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
	
void create_tasks();
task_set_t* tasks_get_main_taskset();

void tasks_run_imu_update();

void tasks_rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motorbool);
task_return_t tasks_set_mav_mode_n_state();
task_return_t tasks_run_stabilisation();
task_return_t tasks_run_gps_update();
task_return_t gumstix_task();
task_return_t run_estimator();
task_return_t tasks_run_navigation_update();
task_return_t tasks_run_barometer_update();
task_return_t send_rt_stats();


#endif /* TASKS_H_ */