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
* \file remote_dsm2.c
*
* This file is the driver for the remote control
*/


#include "remote_dsm2.h"
#include "remote_controller.h"
#include "usart.h"
#include "time_keeper.h"
#include "gpio.h"
#include "sysclk.h"
#include "print_util.h"
#include "delay.h"

static volatile uint8_t packet_byte_counter=0;			///< Declare a counter of bytes in a packet 

Spektrum_Receiver_t spRec1;								///< Declare an object containing the receiver structure for receiver 1
Spektrum_Receiver_t spRec2;								///< Declare an object containing the receiver structure for receiver 2

int16_t channelCenter[16];								///< Declare an array to store the central position of each channel

/**
 * \brief Define the service routine for the spektrum handler interruption
 */
ISR(spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INTLEV_INT1) 
{
	uint8_t c1, c2, i;
	uint8_t channel_encoding, frame_number;
	uint16_t sw;
	uint8_t channel;
	uint32_t now =get_time_ticks() ;
	
	if (REMOTE_UART.csr & AVR32_USART_CSR_RXRDY_MASK) 
	{
		spRec1.duration=now-spRec1.last_time;
		spRec1.last_time=now;
		//putstring(STDOUT, "!");
		//receiveInterruptHandler(&spRec1.receiver);
		if ((spRec1.duration>2500)) 
		{
			buffer_clear(&spRec1.receiver);
		}
		c1=(uint8_t)REMOTE_UART.rhr;
		buffer_put(&spRec1.receiver, c1);
		
		if (buffer_bytes_available(&spRec1.receiver)==16) 
		{
			///< first two bytes are status info
			c1=buffer_get(&spRec1.receiver);
			c2=buffer_get(&spRec1.receiver);
			
			channel_encoding =(c2 & 0x10)>>4;	///< 0 = 11bit, 1 = 10 bit
			frame_number     = c2 & 0x03;		///< 1 = 1 frame contains all channels
			//PORTC.OUT = _BV(5);
			//putstring(STDOUT, "!");
			for (i=1; i<8; i++) 
			{
				c1=buffer_get(&spRec1.receiver);
				c2=buffer_get(&spRec1.receiver);
				sw=(uint16_t)c1<<8 | ((uint16_t)c2);
				//if (c1 & 0x80==0)
				if (channel_encoding==1) 
				{
					///< highest bit is frame 0/1, bits 2-6 are channel number
					channel=((c1&0x80)*8+(c1>>2))&0x0f;
					///< 10 bits per channel
					spRec1.channels[channel]=((int16_t)(sw&0x3ff) - 512)*2;
				} 
				else if (channel_encoding==0) 
				{
					///< highest bit is frame 0/1, bits 3-7 are channel number
					channel=((c1&0x80)*8+(c1>>3))&0x0f;
					///< 11 bits per channel
					spRec1.channels[channel]=((int16_t)(sw&0x7ff) -1024);
				} 
				else 
				{
					///< shouldn't happen!
				}
				//spRec1.channels[i]=sw&0x3ff;
				spRec1.valid=1;
				spRec1.last_update=now;
			} ///< end of For loop
		} ///< end of if (buffer_bytes_available(&spRec1.receiver)==16) 
	} ///< end of if (REMOTE_UART.csr & AVR32_USART_CSR_RXRDY_MASK)		
}

void rc_switch_power(bool on) 
{
	gpio_configure_pin(RECEIVER_POWER_ENABLE_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(RECEIVER_POWER_ENABLE_PIN);
}


void rc_activate_bind_mode() 
{
	int i=0;
	unsigned long cpu_freq=sysclk_get_cpu_hz();
	gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT | GPIO_PULL_DOWN);	
	delay_ms(1);
	gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT| GPIO_INIT_LOW);
	//delay_ms(10);
	///< wait 10 seconds for spektrum to be plugged in
	while ((gpio_get_pin_value(DSM_RECEIVER_PIN) == 0) && (i<10000)) 
	{
		i++;
		delay_ms(1);
	}
	///< wait 100ms after receiver startup
	delay_ms(100);
	///< create 4 pulses with 126us to set receiver to bind mode
	for (i=0; i<3; i++) 
	{
		gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
		cpu_delay_us(113, cpu_freq); 
		gpio_configure_pin(DSM_RECEIVER_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);	
		cpu_delay_us(118, cpu_freq);
	}
}

void rc_init (void) 
{
	static const usart_options_t usart_opt =
	{
		.baudrate     = BAUD_REMOTE,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION},
		{AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION}
	};
	
	for (int i=0; i<16; i++) 
	{
		spRec1.channels[i]=0;
		spRec2.channels[i]=0;
		channelCenter[i]=0;
	}
	spRec1.channels[RC_THROTTLE]=0;
	spRec2.channels[RC_THROTTLE]=0;
	///< USART options.
	 
    ///< Assign GPIO pins to USART_0.
    gpio_enable_module(USART_GPIO_MAP,
                     sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
	
    // Initialize the USART in RS232 mode.
    usart_init_rs232( (&REMOTE_UART), &usart_opt, sysclk_get_cpu_hz());
	INTC_register_interrupt( (__int_handler) &spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1);
	REMOTE_UART.ier=AVR32_USART_IER_RXRDY_MASK;
	//initUART_RX(&spRec1.receiver,  &USARTC1, USART_RXCINTLVL_LO_gc, BSEL_SPEKTRUM);
	//initUART_RX(&spRec2.receiver,  &USARTD0, USART_RXCINTLVL_LO_gc, BSEL_SPEKTRUM);
	rc_switch_power(true);
}

int16_t rc_get_channel(uint8_t index) 
{
	//if (checkReceiver1()<checkReceiver2()) {
		return spRec1.channels[index];
	//} else {
	//	return spRec2.channels[index]-500;
	//}
}

int16_t rc_get_channel_neutral(uint8_t index) {
	int16_t value=rc_get_channel(index)-channelCenter[index];
	///< clamp to dead zone
	if ((value>-DEADZONE)&&(value<DEADZONE))
	{
		value=0;
	}
	return value;
}

void rc_center_channel(uint8_t index)
{
	channelCenter[index]=rc_get_channel(index);
}

int8_t checkReceiver1(void) 
{
	int8_t i;
	uint32_t now = get_time_ticks();
	uint32_t duration=now-spRec1.last_update;
	
	if (spRec1.valid==0) 
	{
		return -2;
	}
	if (duration<100000) 
	{
		return 1;
	} 
	else if (duration<1500000) 
	{
		spRec1.channels[RC_ROLL]=0;	
		spRec1.channels[RC_PITCH]=0;	
		spRec1.channels[RC_YAW]=0;	
		return -1; // brief drop out - hold pattern
	} 
	else 
	{
		spRec1.valid = 0;
		for (i=1; i<8; i++) 
		{
			spRec1.channels[i]=0;			
		}
		spRec1.channels[RC_THROTTLE]=-1000;
		return -2; // fade - fail safe
	}
}

int8_t checkReceiver2(void){
	int8_t i;
	uint32_t now = 0; //TCC0.CNT;
	uint32_t duration = now - spRec2.last_update;
	
	if (spRec2.valid==0) 
	{
		return -2;
	}
	if (duration < 200000) 
	{
		return 1;
	} 
	else if (duration < 500000) 
	{
		return -1; // brief drop out - hold pattern
	} 
	else 
	{
		spRec2.valid = 0;
		for (i=1; i<8; i++) 
		{
			spRec2.channels[i]=0;
		}
		spRec2.channels[RC_THROTTLE]=-1000;
		return -2; ///< fade - fail safe
	}
}

int8_t rc_check_receivers(void) 
{
	return checkReceiver1();// + checkReceiver2();
}

/*
ISR(USARTD0_RXC_vect) {
	uint8_t c1, c2, i;
	uint16_t sw;
	uint16_t now =TCC0.CNT ;
	spRec2.duration=now-spRec2.last_time;
	spRec2.last_time=now;
	receiveInterruptHandler(&spRec2.receiver);

	if ((spRec2.duration>600)&& (spRec2.duration<660)) {
		while (UARTBytesAvailable(&spRec2.receiver)>1) readUARTNonblock(&spRec2.receiver);
	}

	if (UARTBytesAvailable(&spRec2.receiver)==16) {
		//PORTC.OUT = _BV(5);
		for (i=0; i<8; i++) {
			c1=readUARTNonblock(&spRec2.receiver);
			c2=readUARTNonblock(&spRec2.receiver);
			sw=((uint16_t)c1)*256 +((uint16_t)c2);
			//if (c1 & 0x80==0)
			spRec2.channels[(c1 & 0x3c)>>2]=sw&0x3ff;
			//spRec1.channels[i]=sw&0x3ff;
			spRec2.valid=1;
			spRec2.last_update=now;
		}
	}
}
*/

