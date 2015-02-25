/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file dc_motor_ctrl.c
 *
 * This file configures the dc_motor_ctrl UART communication
 */


#include "daler_dc_motor_ctrl.h"
#include "uart_int.h"
#include "spektrum_satellite.h"
#include "turnigy.h"


void daler_dc_motor_ctrl_init(daler_dc_motor_ctrl_t* dc_motor_ctrl, int32_t UID)
{
	// uart setting
	usart_config_t usart_conf_dc_motor_ctrl =
	{
		.mode=UART_IN_OUT,
		.uart_device.uart=(avr32_usart_t *)&AVR32_USART2,
		.uart_device.IRQ=AVR32_USART2_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
			.baudrate     = 57600,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE },
		.rx_pin_map= {AVR32_USART2_RXD_1_PIN, AVR32_USART2_RXD_FUNCTION},
		.tx_pin_map= {AVR32_USART2_TXD_1_PIN, AVR32_USART2_TXD_FUNCTION}
	};
	uart_int_set_usart_conf(UID, &usart_conf_dc_motor_ctrl);
	
	//uart configuration
	uart_int_init(UID);
	uart_int_register_write_stream(uart_int_get_uart_handle(UID), &(dc_motor_ctrl->dc_motor_ctrl_out_stream));
	// Registering streams
	buffer_make_buffered_stream_lossy(&(dc_motor_ctrl->dc_motor_ctrl_in_buffer), &(dc_motor_ctrl->dc_motor_ctrl_in_stream));
	uart_int_register_read_stream(uart_int_get_uart_handle(UID), &(dc_motor_ctrl->dc_motor_ctrl_in_stream));
}


task_return_t daler_dc_motor_ctrl_update(daler_dc_motor_ctrl_t* dc_motor )//, const float wingrons[2])
{
	int i=0;
	int number_of_channels = 8;
	
	int16_t scaled_channel;
	uint8_t scaled_channel_uart_low;
	uint8_t scaled_channel_uart_high;
	
	for (i=0;i<1;i++)
	{
		scaled_channel = 1000.0f + spektrum_satellite_get_channel(i)* 1000.0f * RC_SCALEFACTOR;
		//scaled_channel = 1000.0f + wingrons[i]* 1000.0f * RC_SCALEFACTOR;
		scaled_channel_uart_high = ((scaled_channel >> 8) & 0xff);
		scaled_channel_uart_low = ((scaled_channel >> 0) & 0xff);
		
		dc_motor->dc_motor_ctrl_out_stream.put(dc_motor->dc_motor_ctrl_out_stream.data, scaled_channel_uart_high);
		dc_motor->dc_motor_ctrl_out_stream.put(dc_motor->dc_motor_ctrl_out_stream.data, scaled_channel_uart_low);
	}
	
	
	for (i=1;i<3;i++)
	{
		scaled_channel = 1000.0f * dc_motor->wingrons_angle[i-1];
		//scaled_channel = 1000.0f + wingrons[i]* 1000.0f * RC_SCALEFACTOR;
		scaled_channel_uart_high = ((scaled_channel >> 8) & 0xff);
		scaled_channel_uart_low = ((scaled_channel >> 0) & 0xff);
		
		dc_motor->dc_motor_ctrl_out_stream.put(dc_motor->dc_motor_ctrl_out_stream.data, scaled_channel_uart_high);
		dc_motor->dc_motor_ctrl_out_stream.put(dc_motor->dc_motor_ctrl_out_stream.data, scaled_channel_uart_low);
	}
	
	for (i=3;i<number_of_channels;i++)
	{
		scaled_channel = 1000.0f + spektrum_satellite_get_channel(i)* 1000.0f * RC_SCALEFACTOR;
		//scaled_channel = 1000.0f + wingrons[i]* 1000.0f * RC_SCALEFACTOR;
		scaled_channel_uart_high = ((scaled_channel >> 8) & 0xff);
		scaled_channel_uart_low = ((scaled_channel >> 0) & 0xff);
		
		dc_motor->dc_motor_ctrl_out_stream.put(dc_motor->dc_motor_ctrl_out_stream.data, scaled_channel_uart_high);
		dc_motor->dc_motor_ctrl_out_stream.put(dc_motor->dc_motor_ctrl_out_stream.data, scaled_channel_uart_low);
	}
	
	/*
	for (i=0;i<16;i++)
	{
		dc_motor_ctrl->dc_motor_ctrl_out_stream.put(dc_motor_ctrl->dc_motor_ctrl_out_stream.data, scaled_channels_uart[i]);
	}*/
	
	return TASK_RUN_SUCCESS;
}