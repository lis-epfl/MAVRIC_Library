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
 * \file 	i2c.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Abstract class for i2c peripherals
 *
 ******************************************************************************/

#ifndef I2C_HPP_
#define I2C_HPP_

#include <stdint.h>

class I2c
{
public:

	/**
	 * @brief 	Hardware initialization
	 * 
	 * @return  true Success
	 * @return  false Error
	 */
	virtual bool init(void) = 0;


	/**
	 * @brief 	Test if a chip answers for a given I2C address
	 * 
	 * @param 	address 	Slave adress
	 * 
	 * @return 	true		Slave found
	 * @return 	false		Slave not found
	 */	
	virtual bool probe(uint32_t address) = 0;


	/**
	 * @brief 	Write multiple bytes to a I2C slave device
	 * 
	 * @param 	buffer 		Data buffer
	 * @param 	nbytes 		Number of bytes to write
	 * @param 	address 	Slave adress
	 * 
	 * @return 	true		Data successfully written
	 * @return 	false		Data not written
	 */
	virtual bool write(const uint8_t *buffer, uint32_t nbytes, uint32_t address) = 0;


	/**
	 * @brief 	Read multiple bytes to a I2C slave device
	 * 
	 * @param 	buffer 		Data buffer
	 * @param 	nbytes 		Number of bytes to read
	 * @param 	address 	Slave adress
	 * 
	 * @return 	true		Data successfully read
	 * @return 	false		Data not read
	 */	
	virtual bool read(uint8_t *buffer, uint32_t nbytes, uint32_t address) = 0;
};


#endif /* I2C_HPP_ */