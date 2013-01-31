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
#ifndef _AST_H_
#define _AST_H_

#include "compiler.h"
#include <avr32/io.h>
 
/*! \name Oscillator Types
 */
//! @{
#define AST_OSC_1KHZ     4
#define AST_OSC_GCLK     3
#define AST_OSC_PB       2
#define AST_OSC_32KHZ    1
#define AST_OSC_RC       0
//! @}

/*! \name Predefined PSEL Values
 */
//! @{

//! The PSEL value to set the AST source clock (after the prescaler) to 1 Hz,
//! when using an external 32-kHz crystal.
#define AST_PSEL_32KHZ_1HZ    14

//! The PSEL value to set the AST source clock (after the prescaler) to 1.76 Hz,
//! when using the internal RC oscillator (~ 115 kHz).
#define AST_PSEL_RC_1_76HZ    15

//! @}

/*! \name AST Mode
 */
//! @{
#define AST_MODE_COUNTER  0
#define AST_MODE_CALENDAR 1
//! @}

//! Input when initializing AST in calendar mode.
typedef struct ast_calendar_t
{ 
  union{
    unsigned long            field;
    avr32_ast_calv_t         FIELD;
    /*
    * Description for Calendar Field:
    * typedef struct avr32_ast_calv_t {
    *          //! Field Year    
    *          unsigned int year            : 6;
    *          //! Field Month     
    *          unsigned int month           : 4;
    *          //! Field Day     
    *          unsigned int day             : 5;
    *          //! Field Hour    
    *          unsigned int hour            : 5;
    *          //! Field Minute    
    *          unsigned int min             : 6;
    *          //! Field Seconde    
    *          unsigned int sec             : 6;
    *  } avr32_ast_calv_t;
    *
    */                    
  };
} ast_calendar_t;


/*!
 * \brief This function will initialise the AST module in calendar Mode.
 *        If you use the 32 KHz oscillator, it will enable this module.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param osc_type The oscillator you want to use. If you need a better
 *        accuracy, use the 32 KHz oscillator (i.e. AST_OSC_32KHZ).
 * \param psel The preselector value for the corresponding oscillator (4-bits).
 *        To obtain this value, you can use this formula:
 *        psel = log(Fosc/Fast)/log(2)-1, where Fosc is the frequency of the
 *        oscillator you are using (32 KHz or 115 KHz) and Fast the frequency
 *        desired.
 * \param ast_calendar Startup date
 * \return 1 if the initialisation succeds otherwize it will return 0.
 */
extern int ast_init_calendar(volatile avr32_ast_t *ast, unsigned char osc_type, unsigned char psel,ast_calendar_t ast_calendar);

/*!
 * \brief This function will initialise the AST module in counter Mode.
 *        If you use the 32 KHz oscillator, it will enable this module.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param osc_type The oscillator you want to use. If you need a better
 *        accuracy, use the 32 KHz oscillator (i.e. AST_OSC_32KHZ).
 * \param psel The preselector value for the corresponding oscillator (4-bits).
 *        To obtain this value, you can use this formula:
 *        psel = log(Fosc/Fast)/log(2)-1, where Fosc is the frequency of the
 *        oscillator you are using (32 KHz or 115 KHz) and Fast the frequency
 *        desired.
 * \param ast_counter Startup counter value 
 * \return 1 if the initialisation succeds otherwize it will return 0.
 */
extern int ast_init_counter(volatile avr32_ast_t *ast, unsigned char osc_type, unsigned char psel, unsigned long ast_counter);

/*!
 * \brief Enable the AST.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_enable(volatile avr32_ast_t *ast);

/*!
 * \brief This function sets the AST current calendar value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param ast_calendar Startup date
 */
extern void ast_set_calendar_value(volatile avr32_ast_t *ast, ast_calendar_t ast_calendar);

/*!
 * \brief This function sets the AST current counter value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param ast_counter Startup counter value
 */
extern void ast_set_counter_value(volatile avr32_ast_t *ast, unsigned long ast_counter);

/*!
 * \brief This function returns the AST current calendar value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \return The AST current calendar value.
 */
extern ast_calendar_t ast_get_calendar_value(volatile avr32_ast_t *ast);

/*!
 * \brief This function returns the AST current counter value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \return The AST current counter value.
 */
extern unsigned long ast_get_counter_value(volatile avr32_ast_t *ast);

/*!
 * \brief This function Set the AST alarm0 value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param ast_alarm AST alram0.
 */
extern void ast_set_alarm0_value(volatile avr32_ast_t *ast, U32 ast_alarm);

/*!
 * \brief This function Set the AST alarm1 value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param ast_alarm AST alram1.
 */
extern void ast_set_alarm1_value(volatile avr32_ast_t *ast, U32 ast_alarm);

/*!
 * \brief This function Enable the AST alarm0.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_enable_alarm0(volatile avr32_ast_t *ast);

/*!
 * \brief This function Disable the AST alarm0.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_disable_alarm0(volatile avr32_ast_t *ast);

/*!
 * \brief This function Enable the AST alarm1.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_enable_alarm1(volatile avr32_ast_t *ast);

/*!
 * \brief This function Disable the AST alarm1.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_disable_alarm1(volatile avr32_ast_t *ast);

/*!
 * \brief This function Set the AST periodic0 value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param pir AST periodic0.
 */
void ast_set_periodic0_value(volatile avr32_ast_t *ast, avr32_ast_pir0_t pir);

/*!
 * \brief This function Set the AST periodic1 value.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 * \param pir AST periodic1.
 */
void ast_set_periodic1_value(volatile avr32_ast_t *ast, avr32_ast_pir1_t pir);

/*!
 * \brief This function Enable the AST periodic0.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_enable_periodic0(volatile avr32_ast_t *ast);

/*!
 * \brief This function Disable the AST periodic0.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_disable_periodic0(volatile avr32_ast_t *ast);

/*!
 * \brief This function Enable the AST periodic1.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_enable_periodic1(volatile avr32_ast_t *ast);

/*!
 * \brief This function Disable the AST periodic1.
 * \param ast Base address of the AST (i.e. &AVR32_AST).
 */
extern void ast_disable_periodic1(volatile avr32_ast_t *ast);

#endif  // _AST_H_
