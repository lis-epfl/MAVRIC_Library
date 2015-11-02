/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file 	serial_avr32.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Implementation of serial peripheral for avr32
 *
 ******************************************************************************/

#ifndef SERIAL_AVR32_H_
#define SERIAL_AVR32_H_

#include "serial.hpp"
#include "buffer.hpp"

extern "C"
{
	#include "usart.h"
}

/**
 * \brief 	Enumerate the possible UARTs
 */
typedef enum
{
	AVR32_SERIAL_0 		 	= 0, 
	AVR32_SERIAL_1 			= 1, 
	AVR32_SERIAL_2 			= 2, 
	AVR32_SERIAL_3 			= 3, 
	AVR32_SERIAL_4 			= 4,
	AVR32_SERIAL_MAX_NUMBER = 5,
} serial_avr32_devices_t;


/**
 * \brief 	UART modes
 */
typedef enum
{
	AVR32_SERIAL_OFF 	= 0,
	AVR32_SERIAL_IN 	= 1,
	AVR32_SERIAL_OUT 	= 2,
	AVR32_SERIAL_IN_OUT = 3,
} serial_avr32_mode_t;


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
	serial_avr32_devices_t 	serial_device;
	serial_avr32_mode_t		mode;
	usart_options_t 		options;
	serial_avr32_gpio_map_t rx_pin_map;
	serial_avr32_gpio_map_t tx_pin_map;
} serial_avr32_conf_t;


/**
 * \brief 	Typedef for interrupt handler function
 */
typedef void(*serial_avr32_irq_func_t)(void); 


/**
 * \brief 	Implementation of serial peripheral for avr32
 */
class Serial_avr32: public Serial
{
public:

	/**
	 * \brief  	Initialises the peripheral
	 * 
	 * \param 	config 		Device configuration
	 */
	Serial_avr32(serial_avr32_conf_t config);


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


private:
	serial_avr32_conf_t			config_;			///< Configuration
	volatile avr32_usart_t* 	uart_;				///< Hardware peripheral
	Buffer_tpl<1024>			tx_buffer_;			///< Transmission buffer
	Buffer						rx_buffer_;			///< Reception buffer
	// Buffer_tpl<1024>			rx_buffer_;			///< Reception buffer

	static Serial_avr32*		handlers_[AVR32_SERIAL_MAX_NUMBER]; 	///< Array of 'this' pointers used for interrupt handling


	/**
	 * \brief 		Default interrupt-handling function
	 * 
	 * \details 	This function is called by Serial_avr32::irq(), it is not
	 * 				static, thus has access to object members
	 */
	void irq_handler(void);


	/**
	 * \brief 		Callback function to be called after an interrupt
	 * 
	 * \details 	By default NULL, can be modified via the 'attach' method
	 */
	serial_interrupt_callback_t irq_callback;


	/**
	 * \brief 		Main irq function
	 * 
	 * \details 	This function has to be static in order to be registerable 
	 * 				as interrupt handler. It does internally the dispatch to 
	 * 				call the 'irq_handler' function on the right object instance
	 */
	static void irq0(void); 
	static void irq1(void); 
	static void irq2(void); 
	static void irq3(void); 
	static void irq4(void); 
};


#endif /* SERIAL_AVR32_H_ */