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
 * \file 	gpio.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Abstract class for GPIO peripherals
 *
 ******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

/**
 * \brief 	GPIO direction
 */
typedef enum
{
	GPIO_INPUT  	= 0,	///< Input
	GPIO_OUTPUT 	= 1,	///< Output
} gpio_dir_t;


/**
 * \brief GPIO pull-up / pull-down 
 */
typedef enum
{
	GPIO_PULL_UPDOWN_NONE = 0,		///< No pull up/down
	GPIO_PULL_UPDOWN_UP   = 1,		///< Pull up
	GPIO_PULL_UPDOWN_DOWN = 2,		///< Pull down
} gpio_pull_updown_t;


class Gpio
{
public:

	/**
	 * @brief 	Hardware initialization
	 * 
	 * @return  true 		Success
	 * @return  false 		Error
	 */
	virtual bool init(void) = 0;

	/**
	 * \brief 	Configures the GPIO
	 * 
	 * \param 	dir 	Pin direction (one of enum gpio_dir_t)
	 * \param 	pull 	Pin pull up/down (one of enum gpio_pull_updown_t)
	 * 
	 * \return  success
	 */
	virtual bool configure(gpio_dir_t dir, gpio_pull_updown_t pull) = 0;


	/**
	 * @brief 	Write 1 to the gpio
	 * 
	 * @return 	true		Success
	 * @return 	false		Failed
	 */	
	virtual bool set_high(void) = 0;


	/**
	 * @brief 	Write 0 to the gpio
	 * 
	 * @return 	true		Success
	 * @return 	false		Failed
	 */	
	virtual bool set_low(void) = 0;


	/**
	 * @brief 	Toggle the gpio value 
	 * @details Writes 0 if currently high, writes 1 if currently low
	 * 
	 * @return 	true		Success
	 * @return 	false		Failed
	 */	
	virtual bool toggle(void) = 0;


	/**
	 * @brief 	Write to the gpio pin
	 * 
	 * @param 	level 		Value to write
	 *  
	 * @return 	true		Success
	 * @return 	false		Failed
	 */	
	virtual bool write(bool level) = 0;


	/**
	 * @brief 	Read the current gpio level
	 * 
	 * @return 	Level
	 */	
	virtual bool read(void) = 0;

};


#endif /* GPIO_H_ */