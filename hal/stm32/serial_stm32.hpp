/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file 	serial_stm32.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Implementation of serial peripheral for STM32
 *
 ******************************************************************************/

#ifndef SERIAL_STM32_HPP_
#define SERIAL_STM32_HPP_

#include <libopencm3/stm32/usart.h>

#include "hal/common/serial.hpp"
#include "hal/stm32/gpio_stm32.hpp"
#include "util/buffer.hpp"


/**
 * \brief 	Enumerate the possible UARTs
 */
typedef enum
{
	SERIAL_STM32_1 			= USART1, 
	SERIAL_STM32_2 			= USART2, 
	SERIAL_STM32_3 			= USART3, 
	SERIAL_STM32_4 			= UART4,
	SERIAL_STM32_5 			= UART5,
	SERIAL_STM32_6 			= USART6,
	SERIAL_STM32_7 			= UART7,
	SERIAL_STM32_8 			= UART8,
	SERIAL_STM32_MAX_NUMBER = 8
} serial_stm32_devices_t;


/**
 * \brief 	UART data bits
 */
typedef enum
{
	SERIAL_STM32_DATABITS_8 = 8,
	SERIAL_STM32_DATABITS_9 = 9,
} serial_stm32_databits_t;


/**
 * \brief 	UART stop bits
 */
typedef enum
{
	SERIAL_STM32_STOPBITS_1 	= USART_STOPBITS_1,
	SERIAL_STM32_STOPBITS_0_5 	= USART_STOPBITS_0_5,
	SERIAL_STM32_STOPBITS_2		= USART_STOPBITS_2,
	SERIAL_STM32_STOPBITS_1_5 	= USART_STOPBITS_1_5,
} serial_stm32_stopbits_t;


/**
 * \brief 	UART parity
 */
typedef enum
{
	SERIAL_STM32_PARITY_NONE 	= USART_PARITY_NONE,
	SERIAL_STM32_PARITY_EVEN 	= USART_PARITY_EVEN,
	SERIAL_STM32_PARITY_ODD 	= USART_PARITY_ODD,
} serial_stm32_parity_t;


/**
 * \brief 	UART modes
 */
typedef enum
{
	SERIAL_STM32_MODE_RX 	= USART_MODE_RX,
	SERIAL_STM32_MODE_TX 	= USART_MODE_TX,
	SERIAL_STM32_MODE_TX_RX = USART_MODE_TX_RX,
} serial_stm32_mode_t;


/**
 * \brief 	UART flow control
 */
typedef enum
{
	SERIAL_STM32_FLOWCONTROL_NONE 		= USART_FLOWCONTROL_NONE,
	SERIAL_STM32_FLOWCONTROL_RTS 		= USART_FLOWCONTROL_RTS,
	SERIAL_STM32_FLOWCONTROL_CTS 		= USART_FLOWCONTROL_CTS,
	SERIAL_STM32_FLOWCONTROL_RTS_CTS 	= USART_FLOWCONTROL_RTS_CTS,
} serial_stm32_flowcontrol_t;


/**
 * \brief 	Gpio map
 */
typedef struct
{
	uint8_t  pin;						///< Module pin.
	uint8_t  function;					///< Module function.	
} serial_avr32_gpio_map_t;


/**
 * \brief 	Configuration structure
 */
typedef struct 
{
	serial_stm32_devices_t 		device;
	uint32_t					baudrate;
	serial_stm32_databits_t		databits;
	serial_stm32_stopbits_t		stopbits;
	serial_stm32_parity_t 		parity;
	serial_stm32_mode_t			mode;
	serial_stm32_flowcontrol_t 	flow_control;
	gpio_stm32_port_t 			rx_port;	
	gpio_stm32_pin_t 			rx_pin;
	gpio_stm32_alt_function_t 	rx_af;
	gpio_stm32_port_t 			tx_port;	
	gpio_stm32_pin_t 			tx_pin;
	gpio_stm32_alt_function_t 	tx_af;
} serial_stm32_conf_t;


/**
 * \brief 	Default configuration
 * 
 * \return 	Config structure
 */
static inline serial_stm32_conf_t serial_stm32_default_config();


/**
 * \brief 	Implementation of serial peripheral for avr32
 */
class Serial_stm32: public Serial
{
public:

	/**
	 * \brief  	Initialises the peripheral
	 * 
	 * \param 	config 		Device configuration
	 */
	Serial_stm32(serial_stm32_conf_t config = serial_stm32_default_config());


	/**
	 * \brief 	Hardware initialization
	 * 
	 * \return  true Success
	 * \return  false Error
	 */
	bool init(void);


	/**
	 * \brief 	Test if there are bytes available to read
	 * 
	 * \return 	Number of incoming bytes available
	 */	
	uint32_t readable(void);


	/**
	 * \brief 	Test if there is space available to write bytes
	 * 
	 * \return 	Number of bytes available for writing
	 */	
	uint32_t writeable(void);


	/**
	 * \brief 	Sends instantaneously all outgoing bytes
	 * 
	 * \return 	Number of bytes available for writing
	 */	
	void flush(void);


	/**
	 * \brief 	Attach a function to call after a receive interrupt is generated
	 * 
	 * \details A default handler should be provided by the implementation to 
	 * 			add the incoming data in a buffer, so is not mandatory to call 
	 * 			this method. The function callback will be called after the 
	 * 			interrupt handler
	 * 
	 * \param  	func	 	Pointer to the callback function
	 * 
	 * \return 	true		Success
	 * \return 	false		Failed
	 */
	bool attach(serial_interrupt_callback_t func); 


	/**
	 * \brief 	Write bytes on the serial line
	 * 
	 * \param 	byte 		Outgoing bytes
	 * \param 	size 		Number of bytes to write
	 * 
	 * \return 	true		Data successfully written
	 * \return 	false		Data not written
	 */
	bool write(const uint8_t* bytes, const uint32_t size=1);


	/**
	 * \brief 	Read bytes from the serial line
	 * 
	 * \param 	bytes 		Incoming bytes
	 * \param 	size 		Number of bytes to read
	 * 
	 * \return 	true		Data successfully read
	 * \return 	false		Data not read
	 */	
	bool read(uint8_t* bytes, const uint32_t size=1);


	/**
	 * \brief 		Default interrupt-handling function
	 * 
	 * \details 	This function is called by usartX_isr(), it is not
	 * 				static, thus has access to object members
	 */
	void irq_handler(void);

private:
	serial_stm32_conf_t			config_;			///< Configuration

	Buffer_tpl<1024>			tx_buffer_;			///< Transmission buffer
	Buffer_tpl<1024>			rx_buffer_;			///< Reception buffer

	/**
	 * \brief 		Callback function to be called after an interrupt
	 * 
	 * \details 	By default NULL, can be modified via the 'attach' method
	 */
	serial_interrupt_callback_t irq_callback;
};


static inline serial_stm32_conf_t serial_stm32_default_config()
{
	serial_stm32_conf_t conf = {};
	
	conf.device			= SERIAL_STM32_2;
	conf.baudrate		= 38400;
	conf.databits		= SERIAL_STM32_DATABITS_8;
	conf.stopbits		= SERIAL_STM32_STOPBITS_1;
	conf.parity			= SERIAL_STM32_PARITY_NONE;
	conf.mode			= SERIAL_STM32_MODE_TX_RX;
	conf.flow_control 	= SERIAL_STM32_FLOWCONTROL_NONE;
	conf.rx_port		= GPIO_STM32_PORT_A;	
	conf.rx_pin			= GPIO_STM32_PIN_3;
	conf.rx_af			= GPIO_STM32_AF_7;
	conf.tx_port		= GPIO_STM32_PORT_A;	
	conf.tx_pin			= GPIO_STM32_PIN_2;
	conf.tx_af			= GPIO_STM32_AF_7;

	return conf;
}


#endif /* SERIAL_STM32_HPP_ */