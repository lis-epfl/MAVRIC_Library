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
 * \file led_avr32.cpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * 
 * \brief This file is the driver for the avr32 led
 *
 ******************************************************************************/


#include "led_avr32.hpp"

extern "C"
{
	#include "led.h"
}

Led_avr32::Led_avr32()
{

}

void Led_avr32::on(uint32_t leds)
{
	LED_On(led_id(leds));
	//LED_On(LED1);
	//LED_On(LED2);
}


void Led_avr32::off(uint32_t leds)
{
	LED_Off(led_id(leds));
	//LED_Off(LED1);
	//LED_Off(LED2);
}

void Led_avr32::toggle(uint32_t leds)
{
	LED_Toggle(led_id(leds));
	//LED_Toggle(LED1);
	//LED_Toggle(LED2);
}

uint32_t Led_avr32::led_id(uint32_t leds)
{
	uint32_t led_avr32;

	switch (leds)
	{
		case 1:
			led_avr32 = LED1;
			break;

		case 2:
			led_avr32 = LED2;
			break;

		default:
			led_avr32 = LED1;
	}

	return led_avr32;
}