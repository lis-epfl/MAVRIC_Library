/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief TWI Master driver for AVR UC3.
 *
 * This file defines a useful set of functions for the TWIM interface on AVR UC3
 * devices.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR UC3
 * - Supported devices:  All AVR UC3 devices with an TWIM module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _TWI_MASTER_H_
#define _TWI_MASTER_H_

#include "compiler.h"
#include "sysclk.h"
#include "status_codes.h" 
#include "twim.h"

typedef twi_options_t twi_master_options_t;

static inline int twi_master_setup(volatile avr32_twim_t *twi, twi_master_options_t *opt)
{
  int status;   	
  opt->pba_hz = sysclk_get_pba_hz(); 
	if ((uint32_t)twi == AVR32_TWIM0_ADDRESS) {
		sysclk_enable_pba_module(SYSCLK_TWIM0);	
	} else if ((uint32_t)twi == AVR32_TWIM1_ADDRESS) {
		sysclk_enable_pba_module(SYSCLK_TWIM1);	
	}	 
  status = twi_master_init(twi, (const twi_master_options_t *)opt); 
  return(status);
}

extern int twi_master_write(volatile avr32_twim_t *twi, const twi_package_t *package);
extern int twi_master_read(volatile avr32_twim_t *twi, const twi_package_t *package);
extern void twi_master_enable(volatile avr32_twim_t *twi);
extern void twi_master_disable(volatile avr32_twim_t *twi);
#endif  // _TWI_MASTER_H_
