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
 * \file    rfm22b.hpp
 *
 * \author  MAV'RIC Team
 * \author  Jean-Francois Burnier
 *
 * \brief   This file is the driver for the rfm22b, a RF transceiver
 *
 ******************************************************************************/

#include "hal/common/gpio.hpp"
#include "hal/common/serial.hpp"
#include "hal/common/spi.hpp"

 class Rfm22b : public Serial
 {
 public:

 	/**
     * \brief   Configuration structure for rfm22b
     */
    typedef struct
    {
    	int temp_var;/// add config
    } conf_t;

 	/**
     * \brief   Constructor of the class
     *
     * \param   config          Device configuration
     * \param	spi
     */
    Rfm22b(Spi& spi, Gpio& nss_gpio);//, const conf_t config);
 	/**
     * \brief   Hardware initialization
     *
     * \return  success
     */
    bool init(void);

    /**
     * \brief   Attach a function to call after a receive interrupt is generated
     *
     * \details A default handler should be provided by the implementation to
     *          add the incoming data in a buffer, so is not mandatory to call
     *          this method. The function callback will be called after the
     *          interrupt handler
     *
     * \param   func        Pointer to the callback function
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool attach(serial_interrupt_callback_t func);

    /**
     * \brief   Sends instantaneously all outgoing bytes
     *
     * \return  Number of bytes available for writing
     */
    void flush(void);

    /**
     * \brief   Read register of fm22b

     * \detail  Read data at the specified register,
     *          burst read possible by specifying the
     *          number of bytes to read
     *
     * \param   reg         Register to read from
     * \param   in_data     Incoming data
     * \param   nbytes      Number of bytes to read
     *
     * \return  success
     */
    bool read_reg(uint8_t reg, uint8_t* in_data, uint32_t nbytes = 1);

    /**
     * \brief   Write register of rfm22b
     *
     * \detail  Write data to the specified register,
     *          burst write possible by specifying the
     *          number of bytes to write
     *
     * \param   reg         Register to write to
     * \param   out_data    Outgoing data
     * \param   nbytes      Number of bytes to write
     *
     * \return  success
     */
    bool write_reg(uint8_t reg, uint8_t* out_data, uint32_t nbytes = 1);

    /**
     * \brief   Test if there are bytes available to read
     *
     * \return  Number of incoming bytes available
     */
    uint32_t readable(void);

    /**
     * \brief   Test if there is space available to write bytes
     *
     * \return  Number of bytes available for writing
     */
    uint32_t writeable(void);

    /**
     * \brief   Read bytes from the serial line
     *
     * \param   bytes       Incoming bytes
     * \param   size        Number of bytes to read
     *
     * \return  true        Data successfully read
     * \return  false       Data not read
     */
    bool read(uint8_t* bytes, const uint32_t size = 1);

    /**
     * \brief   Write bytes on the serial line
     *
     * \param   byte        Outgoing bytes
     * \param   size        Number of bytes to write
     *
     * \return  true        Data successfully written
     * \return  false       Data not written
     */
    bool write(const uint8_t* bytes, const uint32_t size = 1);

    /**
     * \brief   Software reset rfm22b
     *
     * \return  success
     */
    bool reset(void);

    bool temp_sens_config(void);
    bool set_normal_carrier_freq(void);
    bool set_datarate(void);
    bool rfm22b_init(void);
    bool to_tx_mode(void);

    // rfm22b register adresses
    static const uint8_t DEVICE_TYPE_REG 	= 0x00;
    static const uint8_t DEVICE_VERSION_REG	= 0x01;
    static const uint8_t DEVICE_STATUS		= 0x02;
    static const uint8_t INTERRUPT_STAT_1	= 0x03;		// Interrupt Status 1
    static const uint8_t INTERRUPT_STAT_2	= 0x04;		// Interrupt Status 2
    static const uint8_t INTERRUPT_EN_1		= 0x05;		// Interrupt Enable 1
    static const uint8_t INTERRUPT_EN_2		= 0x06;		// Interrupt Enable 2
    static const uint8_t OP_FUNC_CNTL_1 	= 0x07; 	// Operating and Function Control 1
    static const uint8_t OP_FUNC_CNTL_2 	= 0x08; 	// Operating and Function Control 2
    static const uint8_t OSC_LOAD_CAP_REG 	= 0x09;
    static const uint8_t CPU_OUTPUT_CLK		= 0x0A;		// Microcontroller Output Clock
    static const uint8_t GPIO0_CONFIG_REG	= 0x0B;		// GPIO 0 Configuration Register
    static const uint8_t GPIO1_CONFIG_REG	= 0x0C;		// GPIO 1 Configuration Register
    static const uint8_t GPIO2_CONFIG_REG	= 0x0D;		// GPIO 2 Configuration Register
    static const uint8_t IO_PORT_CONFIG		= 0x0E;		// I/Os Port Configuration
    static const uint8_t ADC_CONFIG			= 0x0F;		// ADC Configuration
    static const uint8_t ADC_SENS_AMP_OFST	= 0x10;		// ADC Sensor Amplifer Offset
    static const uint8_t TEMP_SENS_CNTL		= 0x12;		// Temperature Sensor Control
    static const uint8_t TEMP_VAL_OFST		= 0x13;		// Temperature Value Offset
    static const uint8_t IF_FILTER_BW		= 0x1C;		// IF Filter Bandwidth
    static const uint8_t AFC_LOOP_GS_OVRRD	= 0x1D;		// AFC Loop Gearshift Override
    static const uint8_t CLK_REC_GS_OVRRD	= 0x1F;		// CLK Recovery Gearshift Override
    static const uint8_t CLK_REC_OVRSMP_RT	= 0x20;		// Clock Recovery Oversampling Ratio
    static const uint8_t RSSI_THRESH_CLR_CH	= 0x27;		// RSSI Threshold for Clear Channel Indicator
    static const uint8_t OOK_COUNTER_VAL_1	= 0x2C;
    static const uint8_t OOK_COUNTER_VAL_2	= 0x2D;
    static const uint8_t SLICER_PEAK_HOLD	= 0x2E;
    static const uint8_t DATA_ACCESS_CNTL	= 0x30;
    static const uint8_t EZMAC_STATUS		= 0x31;		// EzMAC Status
    static const uint8_t HEADER_CNTL_1		= 0x32;		// Header Control 1
    static const uint8_t HEADER_CNTL_2		= 0x33;		// Header Control 2
    static const uint8_t PREAMBLE_LEN_REG	= 0x34;
    static const uint8_t PREAMBLE_DET_CNTL	= 0x35;		// Preamble Detection Control
    static const uint8_t SYNC_WORD_3		= 0x36;
    static const uint8_t SYNC_WORD_2		= 0x37;
    static const uint8_t SYNC_WORD_1		= 0x38;
    static const uint8_t SYNC_WORD_0		= 0x39;
    static const uint8_t TRANSMIT_HEADER_3	= 0x3A;
    static const uint8_t TRANSMIT_HEADER_2	= 0x3B;
    static const uint8_t TRANSMIT_HEADER_1	= 0x3C;
    static const uint8_t TRANSMIT_HEADER_0	= 0x3D;
    static const uint8_t TRANSMIT_PKT_LEN	= 0x3E;		// Transmit Packet Length
    static const uint8_t CHECK_HEADER_3		= 0x3F;
    static const uint8_t CHECK_HEADER_2		= 0x40;
    static const uint8_t CHECK_HEADER_1		= 0x41;
    static const uint8_t CHECK_HEADER_0		= 0x42;
    static const uint8_t HEADER_EN_3		= 0x43;		// Header 3 Enable
    static const uint8_t HEADER_EN_2		= 0x44;		// Header 2 Enable
    static const uint8_t HEADER_EN_1		= 0x45;		// Header 1 Enable
    static const uint8_t HEADER_EN_0		= 0x46;		// Header 0 Enable
    static const uint8_t TX_POWER			= 0x6D;
    static const uint8_t TX_DATA_RATE_1		= 0x6E;
    static const uint8_t TX_DATA_RATE_0		= 0x6F;
    static const uint8_t MOD_MODE_CNTL_1	= 0x70; 	// Modulation Mode Control 1
    static const uint8_t MOD_MODE_CNTL_2	= 0x71; 	// Modulation Mode Control 2
    static const uint8_t FREQ_DEVIATION		= 0x72;
    static const uint8_t FREQ_OFFSET_1		= 0x73;
    static const uint8_t FREQ_OFFSET_2		= 0x74;
    static const uint8_t FREQ_BAND_SEL		= 0x75;		// Frequency Band Select
    static const uint8_t NRML_CARR_FREQ_1	= 0x76;		// Normal Carrier Frequency 1
    static const uint8_t NRML_CARR_FREQ_0	= 0x77;		// Normal Carrier Frequency 0
    static const uint8_t FREQ_HOP_CH_SEL	= 0x79;		// Frequency Hopping Channel Selection
    static const uint8_t FREQ_HOP_STEP_SIZE	= 0x7A;		// Frequency Hopping Step Size
    static const uint8_t TX_FIFO_CNTL_1		= 0x7C;		// Tx FIFO Control 1
    static const uint8_t TX_FIFO_CNTL_2		= 0x7D;		// Tx FIFO Control 1
    static const uint8_t RX_FIFO_CNTL		= 0x7E;		// Rx FIFO Control
    static const uint8_t FIFO_ACCESS_REG	= 0x7F;

    // rfm22b register bits
    static const uint8_t READ_FLAG     	= 0x7F;
    static const uint8_t WRITE_FLAG		= 0x80;
    static const uint8_t DEVICE_TYPE	= 0x08;
    static const uint8_t DEVICE_VERSION = 0x06;

    // Operating and Function Control 1 bits
    static const uint8_t OP_CNTL1_MODE_IDLE_READY 		= 0x01;
    static const uint8_t OP_CNTL1_MODE_IDLE_STANDBY 	= 0x00;
    static const uint8_t OP_CNTL1_MODE_WT_EN			= 0x20;
    static const uint8_t OP_CNTL1_MODE_LBD_EN		 	= 0x40;
    static const uint8_t OP_CNTL1_MODE_PPL_ON 			= 0x02;
    static const uint8_t OP_CNTL1_MODE_TX_ON 			= 0x04;
    static const uint8_t OP_CNTL1_MODE_RX_ON 			= 0x08;
    static const uint8_t OP_CNTL1_SWRESET				= 0x80;

    // Operating and Function Control 2 bits
    static const uint8_t OP_CNTL2_FIFOS_RESET	= 0x03;
    static const uint8_t OP_CNTL2_LOW_DC_EN		= 0x04;

    // ADC Configuration bits
    static const uint8_t ADC_SOURCE_SEL_TEMP_SENSOR = 0x00;
    static const uint8_t ADC_START					= 0x80;

    // Temperature Sensor Control bits
    static const uint8_t TEMP_SENS_CALIB_0	= 0x00;		// -64C to  +64C, 0.5C resolution
    static const uint8_t TEMP_SENS_CALIB_1	= 0x40;		// -40C to  +85C, 1.0C resolution
    static const uint8_t TEMP_SENS_CALIB_2	= 0x80;		//   0C to  +85C, 0.5C resolution
    static const uint8_t TEMP_SENS_CALIB_3	= 0xC0;		// -40F to +216C, 1.0F resolution

    // Modulation Mode Control 1 bits
    static const uint8_t MOD_CNTL1_WHITE_EN	= 0x01;

    // Modulation Mode Control 2 bits
    static const uint8_t MOD_CNTL2_UNMOD_C	= 0x00;
    static const uint8_t MOD_CNTL2_OOK 		= 0x01;
    static const uint8_t MOD_CNTL2_FSK 		= 0x02;
    static const uint8_t MOD_CNTL2_GFSK 	= 0x03;
    static const uint8_t MOD_CNTL2_FD_MSB	= 0x04;
    static const uint8_t MOD_CNTL2_DIR_GPIO	= 0x00;
    static const uint8_t MOD_CNTL2_DIR_SDI	= 0x10;
    static const uint8_t MOD_CNTL2_FIFO		= 0x20;
    static const uint8_t MOD_CNTL2_PN9		= 0x30;
    static const uint8_t MOD_CNTL2_DIS_TX_CLK 	= 0x00;
    static const uint8_t MOD_CNTL2_DCLK_GPIO	= 0x40;
    static const uint8_t MOD_CNTL2_DCLK_SDO		= 0x80;
    static const uint8_t MOD_CNTL2_DCLK_NIRQ	= 0xC0;
    static const uint8_t MOD_CNTL2_INV_EN 		= 0x08;

    // Microcontroller Output Clock bits
    static const uint8_t CPU_OUTPUT_CLK_1MHZ	= 0x06;
    static const uint8_t CPU_OUTPUT_CLK_2MHZ	= 0x05;

    // GPIO 0-1-2 Configuration bits
    static const uint8_t GPIOX_CONFIG_TX_STATE	= 0x12;
    static const uint8_t GPIOX_CONFIG_RX_STATE	= 0x15;
    static const uint8_t GPIOX_CLR_CH_ASSESS	= 0x1C; // Clear Channel Assessment

    // Data Access Control bits
    static const uint8_t ACCESS_TX_PACK_EN		= 0x08;
    static const uint8_t ACCESS_RX_PACK_EN		= 0x80;

    // TX Power bits
    static const uint8_t TX_POWER_LNA_SW		= 0x08;



 private:
 	Spi& 	spi_;	///< SPI peripheral
 	Gpio&	nss_;	///< Slave Select GPIO

 	/**
     * \brief   Select Slave
     */
    void select_slave(void);

    /**
     * \brief   Unselect Slave
     */
    void unselect_slave(void);
 };