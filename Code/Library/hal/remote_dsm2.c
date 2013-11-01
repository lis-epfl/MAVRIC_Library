/*
 * spektrum.c
 *
 *  Created on: Mar 2, 2010
 *      Author: felix
 */
#include "remote_dsm2.h"
#include "remote_controller.h"
#include "usart.h"
#include "time_keeper.h"
#include "gpio.h"
#include "sysclk.h"
#include "print_util.h"

static volatile uint8_t packet_byte_counter=0;

Spektrum_Receiver_t spRec1;
Spektrum_Receiver_t spRec2;

int16_t channelCenter[16];


ISR(spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INTLEV_INT1) {
	uint8_t c1, c2, i;
	uint16_t sw;
	uint32_t now =get_time_ticks() ;
	if (REMOTE_UART.csr & AVR32_USART_CSR_RXRDY_MASK) {
		spRec1.duration=now-spRec1.last_time;
		spRec1.last_time=now;
//		putstring(STDOUT, "!");
		//receiveInterruptHandler(&spRec1.receiver);
		if ((spRec1.duration>2500)) {
			buffer_clear(&spRec1.receiver);
		}
		c1=(uint8_t)REMOTE_UART.rhr;
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

void rc_init (void) {
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
    {AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION}
   ,{AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION}
   };
	int i;
	for (i=0; i<16; i++) {
		spRec1.channels[i]=500;
		spRec2.channels[i]=500;
		channelCenter[i]=0;
	}
	spRec1.channels[RC_THROTTLE]=0;
	spRec2.channels[RC_THROTTLE]=0;
	 // USART options.
	 
    // Assign GPIO pins to USART_0.
    gpio_enable_module(USART_GPIO_MAP,
                     sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
	
    // Initialize the USART in RS232 mode.
    usart_init_rs232( (&REMOTE_UART), &usart_opt, sysclk_get_cpu_hz());
	INTC_register_interrupt( (__int_handler) &spectrum_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1);
	REMOTE_UART.ier=AVR32_USART_IER_RXRDY_MASK;
	//initUART_RX(&spRec1.receiver,  &USARTC1, USART_RXCINTLVL_LO_gc, BSEL_SPEKTRUM);
	//initUART_RX(&spRec2.receiver,  &USARTD0, USART_RXCINTLVL_LO_gc, BSEL_SPEKTRUM);

}
/**/
int16_t rc_get_channel(uint8_t index) {
	//if (checkReceiver1()<checkReceiver2()) {
		return spRec1.channels[index]-500;
	//} else {
	//	return spRec2.channels[index]-500;
	//}
}

int16_t rc_get_channel_neutral(uint8_t index) {
	int16_t value=rc_get_channel(index)-channelCenter[index];
	// clamp to dead zone
	if ((value>-DEADZONE)&&(value<DEADZONE)) value=0;
	return value;
}

void rc_center_channel(uint8_t index){
	channelCenter[index]=rc_get_channel(index);
}

int8_t checkReceiver1() {
	int8_t i;
	uint32_t now = get_time_ticks();
	uint32_t duration=now-spRec1.last_update;
	if (spRec1.valid==0) return -2;
	if (duration<100000) {
		return 1;
	} else
	if (duration<1500000) {
		spRec1.channels[RC_ROLL]=500;	
		spRec1.channels[RC_PITCH]=500;	
		spRec1.channels[RC_YAW]=500;	
		return -1; // brief drop out - hold pattern
		
	} else {
		spRec1.valid = 0;
		for (i=1; i<8; i++) {
			spRec1.channels[i]=500;			
		}
		spRec1.channels[RC_THROTTLE]=0;
		return -2; // fade - fail safe

	}

}

int8_t checkReceiver2(){
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
		spRec2.channels[RC_THROTTLE]=0;
		return -2; // fade - fail safe

	}

}

int8_t rc_check_receivers() {
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

