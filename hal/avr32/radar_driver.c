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
 * \file radar_driver.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief The radar driver
 *
 ******************************************************************************/
 
 
#include "radar_driver.h"


void radar_driver_init(void) 
{
	gpio_configure_pin(RADAR_POWER1_PIN, GPIO_DIR_OUTPUT);	
	gpio_configure_pin(RADAR_POWER2_PIN, GPIO_DIR_OUTPUT);
	radar_driver_switch_power(0,0);
}

void radar_driver_switch_power(int32_t supply1, int32_t supply2) 
{
	if (supply1 == 0) 
	{
		gpio_set_pin_low(RADAR_POWER1_PIN);
	} 
	else 
	{
		gpio_set_pin_high(RADAR_POWER1_PIN);
	}
	if (supply2 == 0) 
	{
		gpio_set_pin_low(RADAR_POWER2_PIN);
	} 
	else 
	{
		gpio_set_pin_high(RADAR_POWER2_PIN);
	}
}



