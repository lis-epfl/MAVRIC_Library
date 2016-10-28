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
#include "drones/lequad.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
}


int main(int argc, char** argv)
{
    uint8_t sysid = 0;
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    mavrimini_conf_t board_config = mavrimini_default_config();
    Mavrimini board(board_config);

    // Board initialisation
    init_success &= board.init();

    // Dummy objects
    File_dummy dummy_file_log;
    File_dummy dummy_file_stat;
    I2c_dummy  i2c_dummy;
    Px4flow_i2c flow_dummy(i2c_dummy);

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using simulated sensors
    LEQuad_dronedome::conf_t mav_config = LEQuad_dronedome::default_config(sysid);
    LEQuad_dronedome mav = LEQuad_dronedome(board.imu,
                        board.sim.barometer(),
                        board.sim.gps(),
                        board.sonar_i2cxl,
                        flow_dummy,
                        board.serial_1,
                        // board.serial_2,
                        board.spektrum_satellite,
                        board.state_display_mavrimini_,
                        board.file_flash,
                        board.battery,
                        board.servo_0,
                        board.servo_1,
                        board.servo_2,
                        board.servo_3,
                        board.servo_4,
                        board.servo_5,
                        board.servo_6,
                        board.servo_7,
                        dummy_file_log,
                        dummy_file_stat,
                        mav_config );

    // initialize MAV
    init_success &= mav.init();

    // -------------------------------------------------------------------------
    // Create simulation
    // -------------------------------------------------------------------------
    // // Simulated servos
    // Pwm_dummy pwm[4];
    // Servo sim_servo_0(pwm[0], servo_default_config_esc());
    // Servo sim_servo_1(pwm[1], servo_default_config_esc());
    // Servo sim_servo_2(pwm[2], servo_default_config_esc());
    // Servo sim_servo_3(pwm[3], servo_default_config_esc());

    // // Simulated dynamic model
    // Dynamic_model_quad_diag sim_model    = Dynamic_model_quad_diag(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
    // Simulation sim                       = Simulation(sim_model);

    // // Simulated battery
    // Adc_dummy    sim_adc_battery = Adc_dummy(11.1f);
    // Battery      sim_battery     = Battery(sim_adc_battery);

    // // Simulated IMU
    // Imu      sim_imu         = Imu(  sim.accelerometer(),
    //                                  sim.gyroscope(),
    //                                  sim.magnetometer() );

    // // set the flag to simulation
    // mav_config.state_config.simulation_mode = HIL_ON;
    // LEQuad mav = LEQuad( MAVLINK_SYS_ID,
    //                              sim_imu,
    //                              sim.barometer(),
    //                              sim.gps(),
    //                              sim.sonar(),
    //                              board.serial_1,                // mavlink serial
    //                              board.spektrum_satellite,
    //                              board.state_display_mavrimini_,
    //                              board.file_flash,
    //                              sim_battery,
    //                              sim_servo_0,
    //                              sim_servo_1,
    //                              sim_servo_2,
    //                              sim_servo_3 ,
    //                              dummy_file_log,
    //                              dummy_file_stat,
    //                              mav_config );

    if (init_success)
    {
      board.green_led.off();
      board.red_led.off();
    }

    print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    return 0;
}







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






















// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>
// #include <libopencm3/stm32/usart.h>

// static void clock_setup(void)
// {
//  // rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
//  rcc_clock_setup_hse_3v3(&hse_25mhz_3v3[CLOCK_3V3_168MHZ]);

//  /* Enable GPIOD clock for LED & USARTs. */
//  rcc_periph_clock_enable(RCC_GPIOC);
//  rcc_periph_clock_enable(RCC_GPIOA);
//  rcc_periph_clock_enable(RCC_GPIOD);

//  /* Enable clocks for USART2. */
//  rcc_periph_clock_enable(RCC_UART4);
// }

// static void usart_setup(void)
// {
//  /* Setup UART4 parameters. */
//  // usart_set_baudrate(UART4, 38400);
//  usart_set_baudrate(UART4, 57600);
//  usart_set_databits(UART4, 8);
//  usart_set_stopbits(UART4, USART_STOPBITS_1);
//  usart_set_mode(UART4, USART_MODE_TX);
//  usart_set_parity(UART4, USART_PARITY_NONE);
//  usart_set_flow_control(UART4, USART_FLOWCONTROL_NONE);

//  /* Finally enable the USART. */
//  usart_enable(UART4);
// }

// static void gpio_setup(void)
// {
//  /* Setup GPIO pin GPIO14 on GPIO port D for LED. */
//  // gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
//  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);

//  /* Setup GPIO pins for UART4 transmit. */
//  gpio_mode_setup(GPIOA, GPIO_MODE_AF,
//                  GPIO_PUPD_NONE,
//                  // GPIO_PUPD_PULLUP,
//                  GPIO0);
//  // gpio_set_output_options(GPIOA,
//  //                      // GPIO_OTYPE_OD,
//  //                      GPIO_OTYPE_PP,
//  //                      GPIO_OSPEED_25MHZ,
//  //                      // GPIO_OSPEED_100MHZ,
//  //                      GPIO2);

//  /* Setup UART4 TX pin as alternate function. */
//  gpio_set_af(GPIOA, GPIO_AF8, GPIO0);
// }

// int main(void)
// {
//  int i, j = 0, c = 0;

//  clock_setup();
//  gpio_setup();
//  usart_setup();

//  /* Blink the LED (PD12) on the board with every transmitted byte. */
//  while (1) {
//      // gpio_toggle(GPIOD, GPIO14);  /* LED on/off */
//      gpio_toggle(GPIOC, GPIO14); /* LED on/off */
//      usart_send_blocking(UART4, c + '0'); /* UART4: Send byte. */
//      c = (c == 9) ? 0 : c + 1;   /* Increment c. */
//      if ((j++ % 80) == 0)
//      {       /* Newline after line full. */
//          usart_send_blocking(UART4, '\r');
//          usart_send_blocking(UART4, '\n');
//      }
//      for (i = 0; i < 3000000; i++) { /* Wait a bit. */
//          __asm__("NOP");
//      }
//  }

//  return 0;
// }








// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>
// #include <libopencm3/stm32/usart.h>
// #include <libopencm3/cm3/nvic.h>

// static void clock_setup(void)
// {
//  /* Enable GPIOD clock for LED & USARTs. */
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
