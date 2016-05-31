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

// #include "boards/mavrimini.hpp"
// #include "sample_projects/LEQuad/central_data.hpp"
// #include "sample_projects/LEQuad/mavlink_telemetry.hpp"
// #include "sample_projects/LEQuad/tasks.hpp"

// extern "C"
// {
// #include "util/print_util.h"
// }

// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>

// int main(int argc, char** argv)
// {
//     uint8_t sysid = 0;
//     bool init_success = true;

//     // -------------------------------------------------------------------------
//     // Create board
//     // -------------------------------------------------------------------------
//     mavrimini_conf_t board_config = mavrimini_default_config();
//     Mavrimini board(board_config);

//     // Board initialisation
//     init_success &= board.init();


//     // Create dummy files
//     File_dummy dummy_file1;
//     File_dummy dummy_file2;

//     // -------------------------------------------------------------------------
//     // Create central data
//     // -------------------------------------------------------------------------
//     // Create central data using simulated sensors
//     central_data_conf_t cd_config = central_data_default_config();

//     Central_data cd = Central_data(sysid,
//                                    board.imu,
//                                    board.sim.barometer(),
//                                    board.sim.gps(),
//                                    board.sim.sonar(),
//                                    board.serial_1,
//                                    // board.serial_2,
//                                    board.spektrum_satellite,
//                                    board.green_led,
//                                    board.file_flash,
//                                    board.battery,
//                                    board.servo_0,
//                                    board.servo_1,
//                                    board.servo_2,
//                                    board.servo_3,
//                                    dummy_file1,
//                                    dummy_file2,
//                                    cd_config );

//     // Init central data
//     init_success &= cd.init();


//     // -------------------------------------------------------------------------
//     // Create tasks and telemetry
//     // -------------------------------------------------------------------------

//     init_success &= mavlink_telemetry_add_onboard_parameters(&cd.mavlink_communication.onboard_parameters, &cd);

//     // // Try to read from flash, if unsuccessful, write to flash
//     // if (onboard_parameters_read_parameters_from_storage(&cd.mavlink_communication.onboard_parameters) == false)
//     // {
//     //     // onboard_parameters_write_parameters_to_storage(&cd.mavlink_communication.onboard_parameters);
//     //     init_success = false;
//     // }

//     init_success &= mavlink_telemetry_init(&cd);

//     cd.state.mav_state = MAV_STATE_STANDBY;

//     init_success &= tasks_create_tasks(&cd);

//     print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

//     // // -------------------------------------------------------------------------
//     // // Main loop
//     // // -------------------------------------------------------------------------

//     // // static uint8_t step = 0;
//     // // while(1)
//     // // {
//     // //  step += 1;

//     // //  if(step%2 == 0)
//     // //  {
//     // //      // board.red_led.toggle();
//     // //  }

//     // //  // gpio_toggle(GPIOA, GPIO2);
//     // //  // usart_send_blocking(UART4, step);
//     // //  // usart_send(UART4, step);
//     // //  // if( step == 80 )
//     // //  // {
//     // //  //  step = 1;
//     // //  //  usart_send(UART4, '\r');
//     // //  //  usart_send(UART4, '\n');
//     // //  // }
//     // //  // usart_enable_tx_interrupt(USART2);
//     // //  board.serial_1.write(&step);

//     // // }

//     if (init_success)
//     {
//         board.green_led.off();
//         board.red_led.off();
//     }

//     while (1 == 1)
//     {
//         //gpio_toggle(GPIOC, GPIO14);
//         // print_util_dbg_print("[HELLO].\r\n");

//          //time_keeper_delay_ms(100);

//         // board.red_led.toggle();
//         scheduler_update(&cd.scheduler);
//     }

//     return 0;
// }







// void uart4_isr(void)
// {
//  // static uint8_t data = 'A';
//  static uint16_t data = 'A';

//  /* Check if we were called because of RXNE. */
//  if (((USART_CR1(UART4) & USART_CR1_RXNEIE) != 0) &&
//      ((USART_SR(UART4) & USART_SR_RXNE) != 0)) {

//      /* Indicate that we got data. */
//      gpio_toggle(GPIOC, GPIO14);

//      /* Retrieve the data from the peripheral. */
//      data = usart_recv(UART4);

//      /* Enable transmit interrupt so it sends back the data. */
//      usart_enable_tx_interrupt(UART4);
//  }

//  /* Check if we were called because of TXE. */
//  if (((USART_CR1(UART4) & USART_CR1_TXEIE) != 0) &&
//      ((USART_SR(UART4) & USART_SR_TXE) != 0)) {

//      /* Put data into the transmit register. */
//      usart_send(UART4, data);

//      /* Disable the TXE interrupt as we don't need it anymore. */
//      usart_disable_tx_interrupt(UART4);
//  }
// }













// void usart2_isr(void)
// {
//  // static uint8_t data = 'A';
//  static uint16_t data = 'A';

//  /* Check if we were called because of RXNE. */
//  if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
//      ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

//      /* Indicate that we got data. */
//      gpio_toggle(GPIOC, GPIO15);

//      /* Retrieve the data from the peripheral. */
//      data = usart_recv(USART2);

//      /* Enable transmit interrupt so it sends back the data. */
//      usart_enable_tx_interrupt(USART2);
//  }

//  /* Check if we were called because of TXE. */
//  if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
//      ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

//      /* Put data into the transmit register. */
//      usart_send(USART2, data);

//      /* Disable the TXE interrupt as we don't need it anymore. */
//      usart_disable_tx_interrupt(USART2);
//  }
// }






















#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "hal/stm32/i2c_stm32.hpp"

#include "drivers/mpu_6050.hpp"
#include "hal/common/time_keeper.hpp"

static void clock_setup_mini(void)
{
 rcc_clock_setup_hse_3v3(&hse_25mhz_3v3[CLOCK_3V3_168MHZ]);
 }

static void clock_setup(void)
{
 rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

static void gpio_setup_mini(void)
{
 /* Setup GPIO pin GPIO14 on GPIO port D for LED. */
 rcc_periph_clock_enable(RCC_GPIOC);
 gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14|GPIO15);
}

static void gpio_setup(void)
{
 /* Setup GPIO pin GPIO14 on GPIO port D for LED. */
 rcc_periph_clock_enable(RCC_GPIOD);
 gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12|GPIO14|GPIO15);
}


int main(void)
{
    bool mini = true;
 // bool mini = false;

 
 if (mini)
 {
     clock_setup_mini();
     gpio_setup_mini();
 }
 else
 {
    clock_setup();
    gpio_setup();
 }

 time_keeper_init();
  

 if(mini)
    gpio_clear(GPIOC, GPIO15);
else
    gpio_clear(GPIOD, GPIO15);
 
i2c_stm32_conf_t i2c_conf = i2c_stm32_default_config();
I2c_stm32 i2c(i2c_conf);
Mpu_6050 mpu(i2c);

// i2c_setup();
i2c.init();

 
for(int j=0; j<4; j++)
{
    if(mini)
        gpio_toggle(GPIOC, GPIO14);
    else
        gpio_toggle(GPIOD, GPIO14);
    
    time_keeper_delay_ms(500);
}

if(!(mpu.init()))
{
    if (mini)
        gpio_set(GPIOC, GPIO15);
    else 
        gpio_set(GPIOD, GPIO15);
}

while (1) 
{
     if (mini)
        gpio_toggle(GPIOC, GPIO14);  /* LED on/off */
     else
        gpio_toggle(GPIOD, GPIO14); /* LED on/off */
     // 
    mpu.update_acc(); 
    
    if (mpu.acc_X() > 0.0f)
    {
        if (mini)
            gpio_set(GPIOC, GPIO15);
        else 
            gpio_set(GPIOD, GPIO12);
    }
    else
    {
        if (mini)
            gpio_clear(GPIOC, GPIO15);
        else 
            gpio_clear(GPIOD, GPIO12);
    }

    time_keeper_delay_ms(500);
 }

 return 0;
}








// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>
// #include <libopencm3/stm32/usart.h>
// #include <libopencm3/cm3/nvic.h>

// static void clock_setup(void)
// {
//  /* Enable GPIOC clock for LED & USARTs. */
//  rcc_periph_clock_enable(RCC_GPIOC);
//  rcc_periph_clock_enable(RCC_GPIOA);

//  /* Enable clocks for USART2. */
//  rcc_periph_clock_enable(RCC_USART2);
// }

// static void usart_setup(void)
// {
//  /* Enable the USART2 interrupt. */
//  nvic_enable_irq(NVIC_USART2_IRQ);

//  /* Setup GPIO pins for USART2 transmit. */
//  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

//  /* Setup GPIO pins for USART2 receive. */
//  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
//  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);

//  /* Setup USART2 TX and RX pin as alternate function. */
//  gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
//  gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

//  /* Setup USART2 parameters. */
//  usart_set_baudrate(USART2, 38400);
//  usart_set_databits(USART2, 8);
//  usart_set_stopbits(USART2, USART_STOPBITS_1);
//  usart_set_mode(USART2, USART_MODE_TX_RX);
//  usart_set_parity(USART2, USART_PARITY_NONE);
//  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

//  /* Enable USART2 Receive interrupt. */
//  usart_enable_rx_interrupt(USART2);

//  /* Finally enable the USART. */
//  usart_enable(USART2);
// }

// static void gpio_setup(void)
// {
//  /* Setup GPIO pin GPIO12 on GPIO port D for LED. */
//  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
//  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
// }

// int main(void)
// {
//  clock_setup();
//  gpio_setup();
//  usart_setup();

//  uint64_t step = 0;

//  while (1)
//  {
//      step += 1;
//      if( step%30000 == 0)
//      {
//          gpio_toggle(GPIOC, GPIO14);
//      }
//      __asm__("NOP");
//  }

//  return 0;
// }

// void usart2_isr(void)
// {
//  static uint8_t data = 'A';

//  /* Check if we were called because of RXNE. */
//  if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
//      ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

//      /* Indicate that we got data. */
//      gpio_toggle(GPIOC, GPIO15);

//      /* Retrieve the data from the peripheral. */
//      data = usart_recv(USART2);

//      /* Enable transmit interrupt so it sends back the data. */
//      usart_enable_tx_interrupt(USART2);
//  }

//  /* Check if we were called because of TXE. */
//  if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
//      ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

//      /* Put data into the transmit register. */
//      usart_send(USART2, data);

//      /* Disable the TXE interrupt as we don't need it anymore. */
//      usart_disable_tx_interrupt(USART2);
//  }
// }
