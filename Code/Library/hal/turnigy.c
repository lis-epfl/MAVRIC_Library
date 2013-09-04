/*
 * turnigy.c
 *
 *  Created on: Aug 27, 2013
 *      Author: ndousse
 */

#include "turnigy.h"
#include "usart.h"
#include "time_keeper.h"
#include "gpio.h"
#include "sysclk.h"
#include "print_util.h"
#include "conf_platform.h"

static volatile uint8_t packet_byte_counter=0;

Turnigy_Receiver_t spRec1;
Turnigy_Receiver_t spRec2;

int16_t channelCenter[16];


ISR(turnigy_handler, AVR32_USART1_IRQ, AVR32_INTC_INTLEV_INT1) {
	uint8_t c1, c2, i;
	uint16_t sw;
	uint32_t now =get_time_ticks() ;
	if (TURNIGY_UART.csr & AVR32_USART_CSR_RXRDY_MASK) {
		spRec1.duration=now-spRec1.last_time;
		spRec1.last_time=now;
//		putstring(STDOUT, "!");
		//receiveInterruptHandler(&spRec1.receiver);
		if ((spRec1.duration>2500)) {
			buffer_clear(&spRec1.receiver);
		}
		c1=(uint8_t)TURNIGY_UART.rhr;
		buffer_put(&spRec1.receiver, c1);
		

		if (buffer_bytes_available(&spRec1.receiver)==16) {
			//PORTC.OUT = _BV(5);
			//putstring(STDOUT, "!");
			for (i=0; i<8; i++) {
				c1=buffer_get(&spRec1.receiver);
				c2=buffer_get(&spRec1.receiver);
				sw=((uint16_t)c1)*256 +((uint16_t)c2);
				//if (c1 & 0x80==0)
				spRec1.channels[(c1 & 0x3c)>>2]=sw&0x3ff;
				//spRec1.channels[i]=sw&0x3ff;
				spRec1.valid=1;
				spRec1.last_update=now;
			}
		}
	}		
}

void turnigy_init (void) {
   static const usart_options_t usart_opt =
   {
     .baudrate     = BAUD_TURNIGY,
     .charlength   = 8,
     .paritytype   = USART_NO_PARITY,
     .stopbits     = USART_1_STOPBIT,
     .channelmode  = USART_NORMAL_CHMODE
   };
   static const gpio_map_t USART_GPIO_MAP =
   {
    {AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION}
   ,{AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION}
   };
	int i;
	for (i=0; i<16; i++) {
		spRec1.channels[i]=500;
		spRec2.channels[i]=500;
		channelCenter[i]=0;
	}
	spRec1.channels[T_THROTTLE]=0;
	spRec2.channels[T_THROTTLE]=0;
	 // USART options.
	 
    // Assign GPIO pins to USART_0.
    gpio_enable_module(USART_GPIO_MAP,
                     sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
	
    // Initialize the USART in RS232 mode.
    usart_init_rs232( (&TURNIGY_UART), &usart_opt, sysclk_get_cpu_hz());
	INTC_register_interrupt( (__int_handler) &turnigy_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1);
	TURNIGY_UART.ier=AVR32_USART_IER_RXRDY_MASK;
	//initUART_RX(&spRec1.receiver,  &USARTC1, USART_RXCINTLVL_LO_gc, BSEL_SPEKTRUM);
	//initUART_RX(&spRec2.receiver,  &USARTD0, USART_RXCINTLVL_LO_gc, BSEL_SPEKTRUM);

}
/**/
int16_t getChannel_turnigy(uint8_t index) {
	int16_t result;
	//if (checkReceiver1()<checkReceiver2()) {
		switch(index)
		{
			case T_ROLL:
				result = spRec1.channels[index]-509;
				break;
			case T_PITCH:
				result = spRec1.channels[index]-511;
				break;
			case T_YAW:
				result = spRec1.channels[index]-530;
				break;
			default:
				result = spRec1.channels[index]-500;
				break;
		}
		return result;
		//return spRec1.channels[index];
		
	//} else {
	//	return spRec2.channels[index]-500;
	//}
}

int16_t getChannelNeutral_turnigy(uint8_t index) {
	int16_t value=getChannel_turnigy(index)-channelCenter[index];
	// clamp to dead zone
	if ((value>-DEADZONE)&&(value<DEADZONE)) value=0;
	return value;
}

void centerChannel_turnigy(uint8_t index){
	channelCenter[index]=getChannel_turnigy(index);
}

int8_t checkReceiver1_turnigy() {
	int8_t i;
	uint32_t now = get_time_ticks();
	uint32_t duration=now-spRec1.last_update;
	if (spRec1.valid==0) return -2;
	if (duration<100000) {
		return 1;
	} else
	if (duration<1500000) {
		spRec1.channels[T_ROLL]=500;	
		spRec1.channels[T_PITCH]=500;	
		spRec1.channels[T_YAW]=500;	
		return -1; // brief drop out - hold pattern
		
	} else {
		spRec1.valid = 0;
		for (i=1; i<8; i++) {
			spRec1.channels[i]=500;			
		}
		spRec1.channels[T_THROTTLE]=0;
		return -2; // fade - fail safe

	}

}

int8_t checkReceiver2_turnigy(){
	int8_t i;
	uint32_t now = 0; //TCC0.CNT;
	uint32_t duration = now - spRec2.last_update;
	if (spRec2.valid==0) return -2;
	if (duration < 200000) {
		return 1;
	} else if (duration < 500000) {
		return -1; // brief drop out - hold pattern
	} else {
		spRec2.valid = 0;
		for (i=1; i<8; i++) {
			spRec2.channels[i]=500;
		}
		spRec2.channels[T_THROTTLE]=0;
		return -2; // fade - fail safe

	}

}

int8_t checkReceivers_turnigy() {
	return checkReceiver1_turnigy();// + checkReceiver2();
}

Control_Command_t get_command_from_turnigy()
{
	Control_Command_t controls;
	controls.rpy[ROLL]= get_roll_from_turnigy();
	controls.rpy[PITCH]= get_pitch_from_turnigy();
	controls.rpy[YAW]= get_yaw_from_turnigy();
	controls.thrust = get_thrust_from_turnigy();
	
	return controls;
}

float get_roll_from_turnigy()
{
	//return getChannelNeutral_turnigy(T_ROLL) * T_SCALEFACTOR;
	return getChannel_turnigy(T_ROLL) * T_SCALEFACTOR;
}

float get_pitch_from_turnigy()
{
	//return -getChannelNeutral_turnigy(T_PITCH) * T_SCALEFACTOR;
	return -getChannel_turnigy(T_PITCH) * T_SCALEFACTOR;
}

float get_yaw_from_turnigy()
{
	//return getChannelNeutral_turnigy(T_YAW) * T_SCALEFACTOR;
	return getChannel_turnigy(T_YAW) * T_SCALEFACTOR;
}

float get_thrust_from_turnigy()
{
	//return min(getChannel(S_THROTTLE)/350.0,board->controls.thrust);
	//return getChannelNeutral_turnigy(T_THROTTLE) * T_SCALEFACTOR;
	return getChannel_turnigy(T_THROTTLE) * T_SCALEFACTOR;
}

void get_channel_mode_turnigy(uint8_t* chanSwitch)
{
	if (getChannel_turnigy(T_GEAR)<0)
	{
		*chanSwitch |= 0x00;
	}else if(getChannel_turnigy(T_GEAR)>0 && getChannel_turnigy(T_ID_MODE)<0){
		*chanSwitch |= 0x01;
	}else if (getChannel_turnigy(T_GEAR)>0 && getChannel_turnigy(T_ID_MODE)>0){
		*chanSwitch |= 0x02;
	}else{
		*chanSwitch |= 0x03;
	}
}