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
 * \file 	serial_dummy.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Dummy implementation for serial peripherals
 *
 ******************************************************************************/

#ifndef SERIAL_DUMMY_H_
#define SERIAL_DUMMY_H_

#include "serial.hpp"
#include <stdint.h>


/**
 * 	Configuration structure
 */
typedef struct
{
	bool flag;			///< Dummy configuration flag
} serial_dummy_conf_t;


/**
 * @brief 	Default configuration
 * 
 * @return 	Config structure
 */
static inline serial_dummy_conf_t serial_dummy_default_config();


class Serial_dummy
{
public:
	/**
	 * @brief 	Initialises the peripheral
	 * 
	 * @param 	config 		Device configuration
	 */
	Serial_dummy(serial_dummy_conf_t config = serial_dummy_default_config());


	/**
	 * @brief 	Hardware initialization
	 * 
	 * @return  true Success
	 * @return  false Error
	 */
	virtual bool init(void);


	/**
	 * @brief 	Test if there are bytes available to read
	 * 
	 * @return 	Number of incoming bytes available
	 */	
	virtual uint32_t readable(void);


	/**
	 * @brief 	Test if there is space available to write bytes
	 * 
	 * @return 	Number of bytes available for writing
	 */	
	virtual uint32_t writeable(void);


	/**
	 * @brief 	Sends instantaneously all outgoing bytes
	 * 
	 * @return 	Number of bytes available for writing
	 */	
	virtual void flush(void);

	/**
	 * @brief 	Write a byte on the serial line
	 * 
	 * @param 	byte 		Outgoing byte
	 * 
	 * @return 	true		Data successfully written
	 * @return 	false		Data not written
	 */
	virtual bool put(const uint8_t byte);


	/**
	 * @brief 	Read one byte from the serial line
	 * 
	 * @param 	byte 		Incoming byte
	 * 
	 * @return 	true		Data successfully read
	 * @return 	false		Data not read
	 */	
	virtual bool get(uint8_t& byte);

private:
	serial_dummy_conf_t config_;
};


/**
 * @brief 	Default configuration
 * 
 * @return 	Config structure
 */
static inline serial_dummy_conf_t serial_dummy_default_config()
{
	serial_dummy_conf_t conf = {};

	conf.flag = true;

	return conf;
}


#endif /* SERIAL_DUMMY_H_ */