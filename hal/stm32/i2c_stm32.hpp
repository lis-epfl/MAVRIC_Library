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
 * \file i2c_avr32.hpp
 * 
 * \author MAV'RIC Team
 *   
 * \brief I2C peripheral driver for STM32
 *
 ******************************************************************************/

#ifndef I2C_STM32_HPP_
#define I2C_STM32_HPP_

#include "i2c.hpp"
#include "gpio_stm32.hpp"

#include <libopencm3/stm32/i2c.h>

/**
 * \brief 	Enumerate the 2 possible I2C
 */
typedef enum
{
	I2C_STM32_1 = I2C1,
	I2C_STM32_2 = I2C2,
	I2C_STM32_3 = I2C3
} i2c_stm32_devices_t;


/**
 * \brief 	Clock CR2 frequency
 */
typedef enum
{
	I2C_STM32_CLK_FREQ_2MHZ	 = I2C_CR2_FREQ_2MHZ,	
	I2C_STM32_CLK_FREQ_3MHZ	 = I2C_CR2_FREQ_3MHZ,	
	I2C_STM32_CLK_FREQ_4MHZ	 = I2C_CR2_FREQ_4MHZ,	
	I2C_STM32_CLK_FREQ_5MHZ	 = I2C_CR2_FREQ_5MHZ,	
	I2C_STM32_CLK_FREQ_6MHZ	 = I2C_CR2_FREQ_6MHZ,	
	I2C_STM32_CLK_FREQ_7MHZ	 = I2C_CR2_FREQ_7MHZ,	
	I2C_STM32_CLK_FREQ_8MHZ	 = I2C_CR2_FREQ_8MHZ,	
	I2C_STM32_CLK_FREQ_9MHZ	 = I2C_CR2_FREQ_9MHZ,	
	I2C_STM32_CLK_FREQ_10MHZ = I2C_CR2_FREQ_10MHZ,	
	I2C_STM32_CLK_FREQ_11MHZ = I2C_CR2_FREQ_11MHZ,	
	I2C_STM32_CLK_FREQ_12MHZ = I2C_CR2_FREQ_12MHZ,	
	I2C_STM32_CLK_FREQ_13MHZ = I2C_CR2_FREQ_13MHZ,	
	I2C_STM32_CLK_FREQ_14MHZ = I2C_CR2_FREQ_14MHZ,	
	I2C_STM32_CLK_FREQ_15MHZ = I2C_CR2_FREQ_15MHZ,	
	I2C_STM32_CLK_FREQ_16MHZ = I2C_CR2_FREQ_16MHZ,	
	I2C_STM32_CLK_FREQ_17MHZ = I2C_CR2_FREQ_17MHZ,	
	I2C_STM32_CLK_FREQ_18MHZ = I2C_CR2_FREQ_18MHZ,	
	I2C_STM32_CLK_FREQ_19MHZ = I2C_CR2_FREQ_19MHZ,	
	I2C_STM32_CLK_FREQ_20MHZ = I2C_CR2_FREQ_20MHZ,	
	I2C_STM32_CLK_FREQ_21MHZ = I2C_CR2_FREQ_21MHZ,	
	I2C_STM32_CLK_FREQ_22MHZ = I2C_CR2_FREQ_22MHZ,	
	I2C_STM32_CLK_FREQ_23MHZ = I2C_CR2_FREQ_23MHZ,	
	I2C_STM32_CLK_FREQ_24MHZ = I2C_CR2_FREQ_24MHZ,	
	I2C_STM32_CLK_FREQ_25MHZ = I2C_CR2_FREQ_25MHZ,	
	I2C_STM32_CLK_FREQ_26MHZ = I2C_CR2_FREQ_26MHZ,	
	I2C_STM32_CLK_FREQ_27MHZ = I2C_CR2_FREQ_27MHZ,	
	I2C_STM32_CLK_FREQ_28MHZ = I2C_CR2_FREQ_28MHZ,	
	I2C_STM32_CLK_FREQ_29MHZ = I2C_CR2_FREQ_29MHZ,	
	I2C_STM32_CLK_FREQ_30MHZ = I2C_CR2_FREQ_30MHZ,	
	I2C_STM32_CLK_FREQ_31MHZ = I2C_CR2_FREQ_31MHZ,	
	I2C_STM32_CLK_FREQ_32MHZ = I2C_CR2_FREQ_32MHZ,	
	I2C_STM32_CLK_FREQ_33MHZ = I2C_CR2_FREQ_33MHZ,	
	I2C_STM32_CLK_FREQ_34MHZ = I2C_CR2_FREQ_34MHZ,	
	I2C_STM32_CLK_FREQ_35MHZ = I2C_CR2_FREQ_35MHZ,	
	I2C_STM32_CLK_FREQ_36MHZ = I2C_CR2_FREQ_36MHZ,	
	I2C_STM32_CLK_FREQ_37MHZ = I2C_CR2_FREQ_37MHZ,	
	I2C_STM32_CLK_FREQ_38MHZ = I2C_CR2_FREQ_38MHZ,	
	I2C_STM32_CLK_FREQ_39MHZ = I2C_CR2_FREQ_39MHZ,	
	I2C_STM32_CLK_FREQ_40MHZ = I2C_CR2_FREQ_40MHZ,	
	I2C_STM32_CLK_FREQ_41MHZ = I2C_CR2_FREQ_41MHZ,	
	I2C_STM32_CLK_FREQ_42MHZ = I2C_CR2_FREQ_42MHZ,	
} i2c_stm32_clk_freq_t;


/**
 * \brief 	Configuration structure
 */
typedef struct
{
	i2c_stm32_devices_t 		device;
	i2c_stm32_clk_freq_t 		clk_freq;
	bool           				tenbit;
	gpio_stm32_port_t 			sda_port;
	gpio_stm32_pin_t        	sda_pin;
	gpio_stm32_alt_function_t 	sda_af;
	gpio_stm32_port_t 			clk_port;
	gpio_stm32_pin_t        	clk_pin;
	gpio_stm32_alt_function_t 	clk_af;
} i2c_stm32_conf_t;


/**
 * \brief 	Default configuration
 * 
 * \return 	Config structure
 */
static inline i2c_stm32_conf_t i2c_stm32_default_config();



/**
 * @brief 	I2C peripheral driver for STM32
 */
class I2c_stm32: public I2c
{
public:
	/**
	 * @brief 	Initialises the peripheral
	 * 
	 * @param 	config 		Device configuration
	 */
	I2c_stm32(i2c_stm32_conf_t config = i2c_stm32_default_config());


	/**
	 * @brief 		Hardware initialization
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
	i2c_stm32_conf_t 		config_;	///< Configuration
};


static inline i2c_stm32_conf_t i2c_stm32_default_config()
{
	i2c_stm32_conf_t conf = {};
	
	conf.device			= I2C_STM32_1;
	conf.clk_freq		= I2C_STM32_CLK_FREQ_25MHZ;
	conf.tenbit			= false;
	conf.sda_port		= GPIO_STM32_PORT_B;	
	conf.sda_pin		= GPIO_STM32_PIN_6;
	conf.sda_af			= GPIO_STM32_AF_4;
	conf.clk_port		= GPIO_STM32_PORT_B;	
	conf.clk_pin		= GPIO_STM32_PIN_7;
	conf.clk_af			= GPIO_STM32_AF_4;

	return conf;
}

#endif /* I2C_STM32_HPP_ */