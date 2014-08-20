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
 * \file data_logging.c
 *
 *  Performs the data logging on the SD card
 */


#include "data_logging.h"
#include "FatFs/diskio.h"
#include "print_util.h"
#include "time_keeper.h"

#include <stdbool.h>

static const char alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

uint32_t loop_count;

void data_logging_init(data_logging_t* data_logging, const imu_t* imu)
{
	loop_count = 0;
	
	data_logging->imu = imu;
	
	data_logging->file_init = false;
	data_logging->continue_writing = true;
	
	data_logging->fr = f_mount(&data_logging->fs, "", 1);
	
	if (data_logging->fr == FR_OK)
	{
		print_util_dbg_print("SD card mounted\r");
	}
	
	data_logging->fr = diskio_open_append(&data_logging->fil,"test.txt");
	
	if (data_logging->fr == FR_OK)
	{
		print_util_dbg_print("File opened\r");
	}
}

bool data_logging_add_header_name(FIL* fp, const char* name);
bool data_logging_add_header_name(FIL* fp, const char* name)
{
	bool init;
	
	print_util_dbg_print("header: returns:");
	
	int32_t res = f_printf(fp,name);
	if (res == EOF)
	{
		init = false;
	}else{
		init = true;
	}
	print_util_dbg_print_num(init,10);
	return init;
}

void data_logging_file_init(data_logging_t* data_logging);
void data_logging_file_init(data_logging_t* data_logging)
{
	bool init;

	print_util_dbg_print("file init:");
	print_util_dbg_print_num(data_logging->fr,10);
	print_util_dbg_print("\r");

	if (data_logging->fr == FR_OK)
	{
		init = data_logging_add_header_name(&data_logging->fil,"time\t");
		
		init = data_logging_add_header_name(&data_logging->fil,"accx\t");
		
		init = data_logging_add_header_name(&data_logging->fil,"accy\t");
		
		init = data_logging_add_header_name(&data_logging->fil,"accz\n");
		
		
		data_logging->file_init = init;
	}
}

void data_logging_put_float(FIL* fp, float c, int32_t after_digits);
void data_logging_put_float(FIL* fp, float c, int32_t after_digits)
{
	int32_t i;
	float num = c;
	
	if (c<0)
	{
		f_puts("-",fp);
		num=-c;
	}

	int32_t whole=abs((int32_t)num);
	float after=(num-(float)whole);

	f_printf(fp,"%d", whole);
	f_puts(".",fp);
	
	for (i=0; i<after_digits; i++)
	{
		after*=10;
		f_printf(fp, "%d",(int32_t)after);
		after=after-(int32_t)after;
	}
}

task_return_t data_logging_run(data_logging_t* data_logging)
{
	if (data_logging->continue_writing)
	{
		if (data_logging->file_init)
		{
			loop_count++;
			if (loop_count == 100)
			{
				data_logging->continue_writing = false;
			}
			
			f_printf(&data_logging->fil, "%d", time_keeper_get_millis());
			f_puts("\t",&data_logging->fil);
			data_logging_put_float(&data_logging->fil,data_logging->imu->scaled_accelero.data[X],5);
			f_puts("\t",&data_logging->fil);
			data_logging_put_float(&data_logging->fil,data_logging->imu->scaled_accelero.data[Y],5);
			f_puts("\t",&data_logging->fil);
			data_logging_put_float(&data_logging->fil,data_logging->imu->scaled_accelero.data[Z],5);
			f_puts("\n",&data_logging->fil);
		}
		else
		{
			data_logging_file_init(data_logging);
		}
	}
	else
	{
		print_util_dbg_print("File closed\r");
		data_logging->fr = f_close(&data_logging->fil);
	}
	return TASK_RUN_SUCCESS;
}