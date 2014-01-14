/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief AST driver for AVR32 UC3.
 *
 * AVR32 Asynchronous Timer (AST) driver module.
 *
 * - Compiler:           GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with an AST and a SCIF module can be used.
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
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. ATMEL grants developer a non-exclusive, limited license to use the Software 
 * as a development platform solely in connection with an Atmel AVR product
 * ("Atmel Product").
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */
#include <avr32/io.h>
#include "compiler.h"
#include "ast.h"

static int ast_is_busy(volatile avr32_ast_t *ast)
{
	return (ast->sr & AVR32_AST_SR_BUSY_MASK) != 0;
}

static int ast_is_clkbusy(volatile avr32_ast_t *ast)
{
	return (ast->sr & AVR32_AST_SR_CLKBUSY_MASK) != 0;
}

int ast_init_calendar(volatile avr32_ast_t *ast,
                      unsigned char osc_type,
                      unsigned char psel,
                      ast_calendar_t ast_calendar)
{

	while (ast_is_clkbusy(ast));
	ast->clock = AVR32_AST_CLOCK_CEN_MASK |
		osc_type << AVR32_AST_CLOCK_CSSEL_OFFSET;

	// Set the new AST configuration
	ast->cr =   AST_MODE_CALENDAR << AVR32_AST_CR_CAL_OFFSET |
		psel << AVR32_AST_CR_PSEL_OFFSET ;

	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));

	// Set the calendar
	ast_set_calendar_value(ast, ast_calendar);

	return 1;
}

int ast_init_counter(volatile avr32_ast_t *ast,
                     unsigned char osc_type,
                     unsigned char psel,
                     unsigned long ast_counter)
{

//	while (ast_is_clkbusy(ast));
	ast->clock = AVR32_AST_CLOCK_CEN_MASK |
		osc_type << AVR32_AST_CLOCK_CSSEL_OFFSET;

	// Set the new AST configuration
	ast->cr =   AST_MODE_COUNTER << AVR32_AST_CR_CAL_OFFSET |
		psel << AVR32_AST_CR_PSEL_OFFSET ;

	// Wait until the ast CTRL register is up-to-date
//	while (ast_is_busy(ast));

	// Set the calendar
	ast_set_counter_value(ast, ast_counter);

	return 1;
}


void ast_enable(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->cr |= AVR32_AST_CR_EN_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}

void ast_set_calendar_value(volatile avr32_ast_t *ast,
                            ast_calendar_t ast_calendar)
{
	// Wait until we can write into the VAL register
	while (ast_is_busy(ast));
	// Set the new val value
	ast->calv = ast_calendar.field;
	// Wait until write is done
	while (ast_is_busy(ast));
}

void ast_set_counter_value(volatile avr32_ast_t *ast, unsigned long ast_counter)
{
	// Wait until we can write into the VAL register
	while (ast_is_busy(ast));
	// Set the new val value
	ast->cv = ast_counter;
	// Wait until write is done
	while (ast_is_busy(ast));
}


ast_calendar_t ast_get_calendar_value(volatile avr32_ast_t *ast)
{
	ast_calendar_t ast_calendar;
	ast_calendar.field = ast->calv;
	return ast_calendar;
}


unsigned long ast_get_counter_value(volatile avr32_ast_t *ast)
{
	return ast->cv;
}

void ast_set_alarm0_value(volatile avr32_ast_t *ast, U32 alarm_value)
{
	// Set the new val value
	ast->ar0 = alarm_value;
}


void ast_set_alarm1_value(volatile avr32_ast_t *ast, U32 alarm_value)
{
	// Set the new val value
	ast->ar1 = alarm_value;
}


void ast_enable_alarm0(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->eve |= AVR32_AST_EVE_ALARM0_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_disable_alarm0(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->evd |= AVR32_AST_EVE_ALARM0_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_enable_alarm1(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->eve |= AVR32_AST_EVE_ALARM1_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_disable_alarm1(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->evd |= AVR32_AST_EVE_ALARM1_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_set_periodic0_value(volatile avr32_ast_t *ast, avr32_ast_pir0_t pir)
{
	// Set the new val value
	ast->PIR0 = pir;
}


void ast_set_periodic1_value(volatile avr32_ast_t *ast, avr32_ast_pir1_t pir)
{
	// Set the new val value
	ast->PIR1 = pir;
}


void ast_enable_periodic0(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->eve |= AVR32_AST_EVE_PER0_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_disable_periodic0(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->evd |= AVR32_AST_EVE_PER0_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_enable_periodic1(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->eve |= AVR32_AST_EVE_PER1_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}


void ast_disable_periodic1(volatile avr32_ast_t *ast)
{
	// Wait until the ast CTRL register is up-to-date
	while (ast_is_busy(ast));
	// Enable the RTC
	ast->evd |= AVR32_AST_EVE_PER0_MASK;
	// Wait until write is done
	while (ast_is_busy(ast));
}
