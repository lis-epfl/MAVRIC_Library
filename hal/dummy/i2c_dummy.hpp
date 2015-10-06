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
 * \file 	i2c_dummy.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Dummy implementation of I2C driver
 *
 ******************************************************************************/

#ifndef I2C_DUMMY_H_
#define I2C_DUMMY_H_

#include "i2c.hpp"


/**
 * 	Configuration structure
 */
typedef struct
{
	bool flag;			///< Dummy configuration flag
} i2c_dummy_conf_t;


/**
 * @brief 	Default configuration
 * 
 * @return 	Config structure
 */
static inline i2c_dummy_conf_t i2c_dummy_default_config();


class I2c_dummy: public I2c
{
public:
	/**
	 * @brief 	Initialises the peripheral
	 * 
	 * @param 	config 		Device configuration
	 */
	I2c_dummy(i2c_dummy_conf_t config = i2c_dummy_default_config());


	/**
	 * @brief 	Hardware initialization
	 * 
	 * @return  true Success
	 * @return  false Error
	 */
	bool init(void);


	/**
	 * @brief 	Test if a chip answers for a given I2C address
	 * 
	 * @param 	address 	Slave adress
	 * 
	 * @return 	True		Slave found
	 * @return 	False		Slave not found
	 */	
	bool probe(uint32_t address);


	/**
	 * @brief 	Write multiple bytes to a I2C slave device
	 * 
	 * @param 	buffer 		Data buffer
	 * @param 	nbytes 		Number of bytes to write
	 * @param 	address 	Slave adress
	 * 
	 * @return 	True		Data successfully written
	 * @return 	False		Data not written
	 */
	bool write(const uint8_t *buffer, uint32_t nbytes, uint32_t address);


	/**
	 * @brief 	Read multiple bytes to a I2C slave device
	 * 
	 * @param 	buffer 		Data buffer
	 * @param 	nbytes 		Number of bytes to read
	 * @param 	address 	Slave adress
	 * 
	 * @return 	True		Data successfully read
	 * @return 	False		Data not read
	 */	
	bool read(uint8_t *buffer, uint32_t nbytes, uint32_t address);


private:
	i2c_dummy_conf_t config_;

};


/**
 * @brief 	Default configuration
 * 
 * @return 	Config structure
 */
static inline i2c_dummy_conf_t i2c_dummy_default_config()
{
	i2c_dummy_conf_t conf = {};

	conf.flag = true;

	return conf;
}


#endif /* I2C_DUMMY_H_ */