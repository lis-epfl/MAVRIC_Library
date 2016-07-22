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
 * \file main.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Main file
 *
 ******************************************************************************/

#include "boards/mavrimini.hpp"
#include "sample_projects/LEQuad/lequad.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
}

extern "C"
{
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
}

static void clock_setup(void)
{
    // Set STM32 to 168 MHz
    // rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);


    // Enable GPIO clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    /* USB clock */
    rcc_periph_clock_enable(RCC_OTGFS);


    time_keeper_init();
}

extern "C"
{
#include "sample_projects/LEQuad/proj_stm32/paparazzi_usb_serial.h"
}

// #############################################################################################
// #############################################################################################
// #############################################################################################
// ####################  MAIN ##################################################################
// #############################################################################################
// #############################################################################################
// #############################################################################################



int main(int argc, char** argv)
{
    // #############################################################################################
    // #############  Clock Setup ##################################################################
    // #############################################################################################
    clock_setup();

    // #############################################################################################
    // #############  USB Setup ##################################################################
    // #############################################################################################
    VCOM_init();

    // #############################################################################################
    // #############  LED Setup ##################################################################
    // #############################################################################################
    gpio_stm32_conf_t led_err_gpio_config;
    gpio_stm32_conf_t led_stat_gpio_config;
    gpio_stm32_conf_t led_rf_gpio_config;

    led_err_gpio_config.port      = GPIO_STM32_PORT_B;
    led_err_gpio_config.pin       = GPIO_STM32_PIN_4;
    led_err_gpio_config.dir      = GPIO_OUTPUT;
    led_err_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;

    led_stat_gpio_config.port     = GPIO_STM32_PORT_B;
    led_stat_gpio_config.pin      = GPIO_STM32_PIN_5;
    led_stat_gpio_config.dir      = GPIO_OUTPUT;
    led_stat_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    led_rf_gpio_config.port    = GPIO_STM32_PORT_B;
    led_rf_gpio_config.pin     = GPIO_STM32_PIN_6;
    led_rf_gpio_config.dir      = GPIO_OUTPUT;
    led_rf_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;

    Gpio_stm32 led_err_gpio(led_err_gpio_config);
    Gpio_stm32 led_stat_gpio(led_stat_gpio_config);
    Gpio_stm32 led_rf_gpio(led_rf_gpio_config);
    Led_gpio led_err(led_err_gpio, false);
    Led_gpio led_stat(led_stat_gpio, false);
    Led_gpio led_rf(led_rf_gpio, false);

    led_err_gpio.init();
    led_stat_gpio.init();
    led_rf_gpio.init();




    // #############################################################################################
    // #############  Blink ! ######################################################################
    // #############################################################################################
    led_err.off();
    led_stat.on();
    led_rf.off();

    while (1)
    {
        time_keeper_delay_ms(10);
        led_rf.toggle();
        time_keeper_delay_ms(10);
        led_stat.toggle();
        time_keeper_delay_ms(10);
        led_err.toggle();

        time_keeper_delay_ms(10);
        VCOM_event();
        // VCOM_putchar(1);
    }

    return 0;
}
