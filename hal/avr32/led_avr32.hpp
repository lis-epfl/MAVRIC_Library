/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file led_avr32.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 *
 * \brief This file is the driver for the avr32 led
 *
 ******************************************************************************/


#ifndef LED_AVR32_HPP_
#define LED_AVR32_HPP_

#include <stdint.h>
#include "hal/common/led.hpp"


typedef enum
{
    LED_AVR32_ID_0 = 0x01,
    LED_AVR32_ID_1 = 0x02,
    LED_AVR32_ID_2 = 0x04,
    LED_AVR32_ID_3 = 0x08
} led_avr32_id_t;


class Led_avr32 : public Led
{
public:
    /**
     * \brief   Constructor
     *
     * \param   id  id of the led (one of led_avr32_id_t enum)
     */
    Led_avr32(led_avr32_id_t id);


    /**
     * \brief   Switch led on
     */
    void on(void);


    /**
     * \brief   Switch led off
     */
    void off(void);


    /**
     * \brief   Toggle led
     */
    void toggle(void);


private:
    led_avr32_id_t id_;             ///< ID of the led
};


#endif /* LED_AVR32_HPP_ */
