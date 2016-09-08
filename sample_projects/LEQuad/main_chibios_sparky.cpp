/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "boards/sparky_chibi/sparky_chibi.hpp"
#include "sample_projects/LEQuad/lequad.hpp"

#include "hal/common/time_keeper.hpp"

#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "drivers/spektrum_satellite.hpp"

#include "drivers/lsm330dlc.hpp"
#include "drivers/hmc5883l.hpp"
#include "drivers/gps_ublox.hpp"
#include "drivers/sonar_i2cxl.hpp"

// #include "hal/chibios/usbcfg.h"

extern "C"
{
    #include "hal_usb.h"
}

#define MAVLINK_SYS_ID 2



char txbuf[] = "Hello World ! \n \r";
char txbuf2[] = "ByeBye! \n \r";
bool flag = false;


/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {

  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {

  (void)uartp;

  uartStopSendI(uartp);
  uartStartSendI(uartp, sizeof(txbuf),txbuf);

  // palSetPad(GPIOD, GPIOD_LED5);
  // chSysLockFromISR();
  // chVTResetI(&vt5);
  // chVTSetI(&vt5, MS2ST(200), led5off, NULL);
  // chSysUnlockFromISR();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {

  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {

  (void)uartp;
  (void)c;
  // /* Flashing the LED each time a character is received.*/
  // palSetPad(GPIOD, GPIOD_LED4);
  // chSysLockFromISR();
  // chVTResetI(&vt4);
  // chVTSetI(&vt4, MS2ST(200), led4off, NULL);
  // chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {

  (void)uartp;
  //
  // /* Flashing the LED each time a character is received.*/
  // palSetPad(GPIOD, GPIOD_LED3);
  // chSysLockFromISR();
  // chVTResetI(&vt3);
  // chVTSetI(&vt3, MS2ST(200), led3off, NULL);
  // chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  38400,
  0,
  USART_CR2_LINEN,
  0
};

#include "hal/chibios/serial_chibios.hpp"


int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    Sparky_chibi::conf_t board_config = Sparky_chibi::default_config();

    // LEDs on STM32F4DISCOVERY
    board_config.gpio_led_err =
    {
        .port  = GPIOD,
        .pin   = GPIOD_PIN12_LED4
    };
    board_config.gpio_led_stat =
    {
        .port  = GPIOD,
        .pin   = GPIOD_PIN13_LED3
    };

    Sparky_chibi board(board_config);

    // Board initialisation
    init_success &= board.init();

    // -------------------------------------------------------------------------
    // Create devices that are not implemented in board yet
    // -------------------------------------------------------------------------
    // Dummy peripherals
    Serial_dummy        serial_dummy;
    I2c_dummy           i2c_dummy;
    File_dummy          file_dummy;
    Gpio_dummy          gpio_dummy;
    Spektrum_satellite  satellite_dummy(serial_dummy, gpio_dummy, gpio_dummy);
    Led_gpio            led_dummy(gpio_dummy);

    // -------------------------------------------------------------------------
    // Create simulation
    // -------------------------------------------------------------------------
    // Simulated servos
    Pwm_dummy pwm[8];
    Servo sim_servo_0(pwm[0], servo_default_config_esc());
    Servo sim_servo_1(pwm[1], servo_default_config_esc());
    Servo sim_servo_2(pwm[2], servo_default_config_esc());
    Servo sim_servo_3(pwm[3], servo_default_config_esc());
    Servo sim_servo_4(pwm[4], servo_default_config_esc());
    Servo sim_servo_5(pwm[5], servo_default_config_esc());
    Servo sim_servo_6(pwm[6], servo_default_config_esc());
    Servo sim_servo_7(pwm[7], servo_default_config_esc());

    // Simulated dynamic model
    Dynamic_model_quad_diag     sim_model(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
    Simulation                  sim(sim_model);

    // Simulated battery
    Adc_dummy   sim_adc_battery(11.1f);
    Battery     sim_battery(sim_adc_battery);

    // Simulated IMU
    Imu     sim_imu( sim.accelerometer(),
                     sim.gyroscope(),
                     sim.magnetometer() );


    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    LEQuad mav = LEQuad( sim_imu,
                         sim.barometer(),
                         sim.gps(),
                         sim.sonar(),
                         serial_dummy,                // mavlink serial
                         satellite_dummy,
                         board.state_display_,
                         file_dummy,
                         sim_battery,
                         sim_servo_0,
                         sim_servo_1,
                         sim_servo_2,
                         sim_servo_3 ,
                         sim_servo_4,
                         sim_servo_5,
                         sim_servo_6,
                         sim_servo_7 ,
                         file_dummy,
                         file_dummy,
                         mav_config );


    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    // mav.loop();

    State_display& disp = board.state_display_;
    // I2c_chibios& i2c = board.i2c1_;

    /**
    * Prepares the barometer
    */
    static uint8_t rxbuf[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static uint8_t txbuf[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    // const uint8_t ms5611_addr       = 0x77;
    // const uint8_t COMMAND_RESET     = 0x1E;  ///< Reset sensor
    // const uint8_t COMMAND_GET_CALIBRATION          = 0xA2;  ///< Get 16bytes factory configuration from PROM


    /*
     * Initializes a serial-over-USB CDC driver.
     */
    // sduObjectInit(&SDU1);
    // sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    // usbDisconnectBus(serusbcfg.usbp);
    // time_keeper_delay_ms(1500);
    // usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);

    // usbDisconnectBus(&USBD1);
    // time_keeper_delay_ms(1500);
    // usbStart(&USBD1, &usbcfg);
    // usbConnectBus(&USBD1);


    time_keeper_delay_ms(1500);

    // static SerialConfig uartCfg =
    // {
    //     9600, // bit rate
    // };
    //
    // palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // used function : USART3_TX
    // palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7)); // used function : USART3_RX
    // sdStart(&SD3, &uartCfg); // starts the serial driver with uartCfg as a config
    // char data[] = "Hello World ! \n \r";
    // char data2[] = "ByeBye World ! \n \r";

    /*
     * Activates the UART driver 2, PA2(TX) and PA3(RX) are routed to USART2.
     */
    // uartStart(&UARTD2, &uart_cfg_1);
    // palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
    // palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
    //
    // uartStopReceive(&UARTD2);
    // uartStopSend(&UARTD2);
    // // uartStartReceive(&UARTD2, 16, buffer);


    Serial_chibios serial({Serial_chibios::SERIAL_1, &UARTD1, 38400});
    serial.init();

    while (true)
    {
        disp.update();
        time_keeper_delay_ms(100);

        serial.write(txbuf, sizeof(txbuf));
        serial.write(txbuf, sizeof(txbuf));
        serial.write(txbuf, sizeof(txbuf));
        serial.write(txbuf, sizeof(txbuf));
        // uartStopSend(&UARTD2);
        // uartStartSendI(&UARTD2, sizeof(txbuf), txbuf);
        //
        for (size_t i = 0; i < 10; i++)
        {
            Servo& servo = board.servo_[i];
            servo.write(1.0f);
        }
        serial.write(txbuf, sizeof(txbuf));
        serial.write(txbuf, sizeof(txbuf));
        serial.write(txbuf, sizeof(txbuf));
        serial.write(txbuf, sizeof(txbuf));
        // uartStopSend(&UARTD2);
        // uartStartSendI(&UARTD2, sizeof(txbuf), txbuf);
        //
        for (size_t i = 0; i < 10; i++)
        {
            Servo& servo = board.servo_[i];
            servo.write(0.0f);
        }

        // uartStopSend(&UARTD2);
        // uartStartSend(&UARTD2, sizeof(data2), data2);
        // for (size_t i = 0; i < 10; i++)
        // {
        //     Servo& servo = board.servo_[i];
        //     servo.write(-1.0f);
        // }

        // chnWrite(&SD3, (uint8_t *) data, strlen(data));
        // // sdAsynchronousWrite(&SD3, (uint8_t *) data, strlen(data));
        // for (size_t i = 0; i < 10; i++)
        // {
        //     Servo& servo = board.servo_[i];
        //     servo.write(1.0f);
        // }
        // chnWrite(&SD3, (uint8_t *) data2, strlen(data));
        // // sdAsynchronousWrite(&SD3, (uint8_t *) data2, strlen(data));
        // for (size_t i = 0; i < 10; i++)
        // {
        //     Servo& servo = board.servo_[i];
        //     servo.write(-1.0f);
        // }



        // time_keeper_delay_ms(1);
        // while (true) {
            // msg_t msg = usbTransmit(&USBD1, USBD2_DATA_REQUEST_EP,
            //                       txbuf, sizeof (txbuf) - 1);
            // if (msg == MSG_RESET)
            // {
            //     time_keeper_delay_ms(500);
            // }


        // pwm1.set_pulse_width_us(1000);
        // pwm1.set_period_us(20000);
        // servo.write(-1.0f);


        // for (size_t i = 0; i < 10; i++)
        // {
        //     Servo& servo = board.servo_[i];
        //     servo.write(-1.0f);
        // }
        // time_keeper_delay_ms(20);
        // for (size_t i = 0; i < 10; i++)
        // {
        //     Servo& servo = board.servo_[i];
        //     servo.write(0.0f);
        // }
        // time_keeper_delay_ms(20);
        // for (size_t i = 0; i < 10; i++)
        // {
        //     Servo& servo = board.servo_[i];
        //     servo.write(1.0f);
        // }


        // }
        // chnWrite(&SDU1, (uint8_t *)"Hello World!\r\n", 14);


        // // Reset sensor
        // txbuf[0] = COMMAND_RESET;
        // i2c.write(txbuf, 1, ms5611_addr);
        //
        // // Let time to the sensor to initialize
        // time_keeper_delay_ms(20);
        //
        // // Read calibration data
        // for (size_t i = 0; i < 6; i++)
        // {
        //     txbuf[0] = COMMAND_GET_CALIBRATION + i * 2;
        //     i2c.transfer(txbuf, 1, rxbuf, 2, ms5611_addr);
        // }


        // while (true)
        // {
        //     if (SDU1.config->usbp->state == USB_ACTIVE)
        //     {
        //         static uint8_t buf[] = "0123456789abcdef\n";
        //
        //         chnRead(&SDU1, buf, sizeof buf - 1);
        //         chnWrite(&SDU1, buf, sizeof buf - 1);
        //     }
        // }

    }

     return 0;
 }
