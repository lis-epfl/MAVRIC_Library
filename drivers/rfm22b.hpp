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
#include "hal/common/led_gpio.hpp"
#include "hal/common/serial.hpp"
#include "hal/common/spi.hpp"

static const uint8_t crc_table[256] = {
0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

static const uint8_t packet_time[]      = { 80, 40, 25, 15, 13, 10, 8, 6, 5 };
static const uint8_t packet_time_ppm[]  = { 26, 25, 25, 15, 13, 10, 8, 6, 5 };
static const uint8_t num_channels[]     = { 4, 4, 4, 6, 8, 8, 10, 12, 16 };

static const uint8_t reg_1C[] = { 0x01, 0x05, 0x06, 0x95, 0x95, 0x81, 0x88, 0x8B, 0x8D };   // rfm22_if_filter_bandwidth

static const uint8_t reg_1D[] = { 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40 };   // rfm22_afc_loop_gearshift_override
static const uint8_t reg_1E[] = { 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x02 };   // rfm22_afc_timing_control

static const uint8_t reg_1F[] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 };   // rfm22_clk_recovery_gearshift_override
static const uint8_t reg_20[] = { 0xA1, 0xD0, 0x7D, 0x68, 0x5E, 0x78, 0x5E, 0x3F, 0x2F };   // rfm22_clk_recovery_oversampling_ratio
static const uint8_t reg_21[] = { 0x20, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02 };   // rfm22_clk_recovery_offset2
static const uint8_t reg_22[] = { 0x4E, 0x9D, 0x06, 0x3A, 0x5D, 0x11, 0x5D, 0x0C, 0xBB };   // rfm22_clk_recovery_offset1
static const uint8_t reg_23[] = { 0xA5, 0x49, 0x25, 0x93, 0x86, 0x11, 0x86, 0x4A, 0x0D };   // rfm22_clk_recovery_offset0
static const uint8_t reg_24[] = { 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x06, 0x07 };   // rfm22_clk_recovery_timing_loop_gain1
static const uint8_t reg_25[] = { 0x34, 0x88, 0x77, 0x29, 0xE2, 0x90, 0xE2, 0x1A, 0xFF };   // rfm22_clk_recovery_timing_loop_gain0

static const uint8_t reg_2A[] = { 0x1E, 0x24, 0x28, 0x3C, 0x3C, 0x50, 0x50, 0x50, 0x50 };   // rfm22_afc_limiter .. AFC_pull_in_range = �AFCLimiter[7:0] x (hbsel+1) x 625 Hz

static const uint8_t reg_58[] = { 0x80, 0x80, 0x80, 0x80, 0x80, 0xC0, 0xC0, 0xC0, 0xED };   // rfm22_cpcuu
static const uint8_t reg_69[] = { 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };   // rfm22_agc_override1
static const uint8_t reg_6E[] = { 0x4E, 0x9D, 0x08, 0x0E, 0x10, 0x19, 0x20, 0x31, 0x41 };   // rfm22_tx_data_rate1
static const uint8_t reg_6F[] = { 0xA5, 0x49, 0x31, 0xBF, 0x62, 0x9A, 0xC5, 0x27, 0x89 };   // rfm22_tx_data_rate0

static const uint8_t reg_70[] = { 0x2C, 0x2C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C };   // rfm22_modulation_mode_control1
static const uint8_t reg_71[] = { 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23 };   // rfm22_modulation_mode_control2

static const uint8_t reg_72[] = { 0x30, 0x48, 0x48, 0x48, 0x48, 0x60, 0x90, 0xCD, 0x0F };   // rfm22_frequency_deviation

// xtal 10 ppm, 434MHz
static const uint32_t data_rate[] = {
    9600,           // 9.6  kbps, 433 HMz, 30  khz freq dev
    19200,          // 19.2 kbps, 433 MHz, 45  khz freq dev
    32000,          // 32   kbps, 433 MHz, 45  khz freq dev
    57600,          // 57.6 kbps, 433 MHz, 45  khz freq dev
    64000,          // 64   kbps, 433 MHz, 45  khz freq dev
    100000,         // 100  kbps, 433 MHz, 60  khz freq dev
    128000,         // 128  kbps, 433 MHz, 90  khz freq dev
    192000,         // 192  kbps, 433 MHz, 128 khz freq dev
    256000,         // 256  kbps, 433 MHz, 150 khz freq dev
};

 class Rfm22b : public Serial
 {
 public:

    /**
     * \brief   Union for device status
     */
    typedef union
    {
        struct
        {
            uint8_t state       : 2;
            bool reserved1;
            bool reserved2;
            bool header_error   : 1;
            bool rx_fifo_empty  : 1;
            bool fifo_underflow : 1;
            bool fifo_overflow  : 1;

        };
        uint8_t raw;
    } device_status_reg_t;

    /**
     * \brief   Union for interrupt status 1
     */
    typedef union
    {
        struct
        {
            bool crc_error                  : 1;
            bool valid_packet_received      : 1;
            bool packet_sent_interrupt      : 1;
            bool external_interrupt         : 1;
            bool rx_fifo_almost_full        : 1;
            bool tx_fifo_almost_empty       : 1;
            bool tx_fifo_almost_full        : 1;
            bool fifo_underoverflow_error   : 1;

        };
        uint8_t raw;
    } interrupt_status1_t;

    /**
     * \brief   Union for interrupt status 2
     */
    typedef union
    {
        struct
        {
            bool poweron_reset              : 1;
            bool chip_ready                 : 1;
            bool low_battery_detect         : 1;
            bool wakeup_timer               : 1;
            bool rssi_above_threshold       : 1;
            bool invalid_preamble_detected  : 1;
            bool valid_preamble_detected    : 1;
            bool sync_word_detected         : 1;
        };
        uint8_t raw;
    } interrupt_status2_t;

    /**
     * \brief   Union for ezMAC status
     */
    typedef union
    {
        struct
        {
            bool packet_sent            : 1;
            bool packet_transmitting    : 1;
            bool crc_error              : 1;
            bool valid_packet_received  : 1;
            bool packet_receiving       : 1;
            bool packet_searching       : 1;
            bool crc_is_all_ones        : 1;
            bool reserved;
        };
        uint8_t raw;
    } ezmac_status_reg_t;

    /**
     * \brief   Enum for transaction states
     */
    typedef enum
    {
        TRANS_STATE_INITIALIZING,
        TRANS_STATE_TRANSITION,     // Transition between RX and TX and vice versa (I guess)
        TRANS_STATE_RX_WAIT,
        TRANS_STATE_RX_WAIT_SYNC,
        TRANS_STATE_RX_MODE,
        TRANS_STATE_TX_MODE,
        TRANS_STATE_TRANSMITTING,

        TRANS_STATE_NUM_STATES // Must be last
    } transaction_state_t;

    /**
     * \brief   Status of the device
     */
    typedef struct
    {
        device_status_reg_t device_status_reg;
        ezmac_status_reg_t  ezmac_status;
        interrupt_status1_t int_status_1;
        interrupt_status2_t int_status_2;
    } device_status_t;

 	/**
     * \brief   Configuration structure for rfm22b
     */
    typedef struct
    {
    	int temp_var;/// add config!!!
    } conf_t;

 	/**
     * \brief   Constructor of the class
     *
     * \param   config          Device configuration
     * \param	spi
     */
    Rfm22b(Spi& spi, Gpio& nss_gpio, Gpio& nirq_gpio, Led_gpio& led_irq);//, const conf_t config);
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


    bool init_2(void); // TODO: change name!!

    /**
     * \brief   Configure temperature sensor and ADC
     *
     * \return  success
     */
    bool temp_sens_config(void);

    /**
     * \brief   Set datarate specified by datarate_
     *
     * \details Set the datarate to datarate_, enables data whitening
     *          and manchester coding (??) by default.
     *          Warning: cannot set datarate to any desired datarate
     *          because of the complexity of the configurations. Only
     *          a set of datarate is possible to configugure: see
     *          data_rate[] array.
     *
     * \return  success
     */
    bool set_datarate(void);

    /**
     * \brief   Set nominal carrier frequency and frequency hopping step size
     *
     * \return  success
     */
    bool set_nominal_carrier_frequency(uint32_t frequency_hz, uint8_t init_chan);

    /**
     * \brief   Set modulation type
     *
     * \details There are three different modulation type:
     *          OOK, FSK and GFSK. GFSK is always chosen
     *
     * \return  success
     */
    bool set_modulation_type(void);

    /**
     * \brief   Set modulation data source
     *
     * \details There are three different data source
     *          to choose from:
     *          - FIFO:                 Stores data into FIFO
     *          - Direct Data Source:   Data goes directly to microcontroller
     *          - PN9:                  Random data generated (used for testing)
     *          FIFO always chosen.
     *
     * \return  success
     */
    bool set_modulation_data_source(void);

    /**
     * \brief   Set data clock configuration
     *
     * \details In Direct Data Mode, there are two possible of data acquiring:
     *          - asynchronous
     *          - synchronus
     *          Asynchronous mode only works for FSK and OOK modulation types.
     *          There are different ways to get a clock signal to the device:
     *          - From GPIOX (0,1 or 2) and they should also be programmed.
     *          - From SDO pin (only when NSEL pin is high).
     *          - From nIRQ pin.
     *
     * \return  success
     */
    bool set_data_clock_configuration(void);

    /**
     * \brief   Set transmission power
     *
     * \param   power_dBm   Power in dBm (from 1dBm to 20dBm)
     *
     * \return  success
     */
    bool set_transmission_power(uint8_t power_dBm);

    /**
     * \brief   Configure GPIO 0, 1 and 2  
     *
     * \return  success
     */
    bool set_gpio_function(void);

    /**
     * \brief   Set transmit the transmit header
     *
     * \details Transmit header is set to contain the id of the
     *          coordinator or the device depending if the device
     *          is communicating with a coordinator or not
     *
     * \return  success
     */
    bool set_transmit_header(void);

    /**
     * \brief   Clear TX FIFO  
     *
     * \return  success
     */
    bool clear_tx_fifo(void);

    /**
     * \brief   Clear RX FIFO  
     *
     * \return  success
     */
    bool clear_rx_fifo(void);

    /**
     * \brief   Get Received Strength Signal Indicator value
     *
     * \param   rssi    pointer to the rssi value
     *
     * \return  success
     */
    bool get_rssi(uint8_t* rssi);

    /**
     * \brief   Set the preamble detection length
     *
     * \param   n_nibbles   Detection search for a preamble
     *                      pattern of length n_nibbles (in
     *                      nibbles)
     *
     * \return  success
     */
    bool set_preamble_detection(uint8_t n_nibbles);

    /**
     * \brief   Set the length of the transmit header
     *
     * \details there are five different length possible:
     *          - 0 byte  (no header)
     *          - 1 byte  (Header 3)
     *          - 2 bytes (Header 3 then 2)
     *          - 3 bytes (Header 3 then 2 then 1)
     *          - 4 bytes (Header 3 then 2 then 1 then 0)
     *
     * \param   length  length to set (0 = 0 bytes, 1 = 1 bytes, ...)
     *
     * \return  success
     */
    bool set_header_length(uint8_t length);

    /**
     * \brief   Set the bytes of the header to be checked upon receipt
     *
     * \details Each bit defines if the corresponding byte is to be checked,
     *          i.e. 0b0101 means check header 0 and 2 but not 1 and 3.
     *
     * \return  success
     */
    bool set_header_check(void);

    /**
     * \brief   Enable the broadcast address check
     *
     * \details Each bit defines if the corresponding byte is to be checked,
     *          i.e. 0b0101 means check header 0 and 2 but not 1 and 3.
     *
     * \return  success
     */
    bool set_header_broadcast_address_check(void);

    /**
     * \brief   Set the expected header
     *
     * \details Set the expected header for a receipt (should match
     *          the transmit header)
     *
     * \return  success
     */
    bool set_check_header(void);

    /**
     * \brief   Get the received header
     *
     * \param   rx_header   Pointer to the received header
     *
     * \return  success
     */
    bool get_received_header(uint8_t* rx_header);

    /**
     * \brief   Set the length of the syncword
     *
     * \details there are four different length possible:
                - 1 byte  (Syncword byte 3)
                - 2 bytes (Syncword byte 3 and 2)
                - 3 bytes (Syncword byte 3 and 2 and 1)
                - 4 bytes (Syncword byte 3 and 2 and 1 and 0)
     *
     * \param   length  length of the syncword (=0 is 1 byte, =1 is 2 bytes, ...)
     *
     * \return  success
     */
    bool set_syncword_length(uint8_t length);

    /**
     * \brief   Get the transmited header
     *
     * \param   tx_header   Pointer to the transmited header
     *
     * \return  success
     */
    bool get_transmit_header(uint8_t* tx_header);

    /**
     * \brief   Enable header check
     *
     * \details Unlike header check, its enable the bits to check
     *
     * \return  success
     */
    bool header_enable(void);

    /**
     * \brief   Interrupt enable
     *
     * \details If at least one bit is set, the register (interrupt status 1 or 2)
     *          will be reset each time it is read. If left empty then act like a
     *          common status register
     *
     * \param   in1en   interrupt status 1 enable, enables corresponding bits
     * \param   in2en   interrupt statis 2 enable, enables corresponding bits
     *
     * \return  success
     */
    bool interrput_enable(uint8_t in1en, uint8_t in2en);

    /**
     * \brief   Set RSSI offset
     *
     * \details each increment is an increment of 4dBm
     *
     * \param   offset  offset to set
     *
     * \return  success
     */
    bool set_rssi_offset(uint8_t offset);

    /**
     * \brief   Set RSSI threshold
     *
     * \details An interruption can be programmed if rssi value is above
     *          this threshold
     *
     * \param   threshold_dBm   threshold in dBm
     *
     * \return  success
     */
    bool set_rssi_threshold(uint8_t threshold_dBm);

    /**
     * \brief   Get the battery level
     *
     * \details get the battery level in Volt
     *
     * \param   battery_level   pointer to the battery level
     *
     * \return  success
     */
    bool get_battery_level(float* battery_level);

    /**
     * \brief   Set the Low Battery Detection threshold
     *
     * \details An interruption can be programmed if LBD value is below
     *          this threshold
     *
     * \param   threshold_V   threshold in Volt
     *
     * \return  success
     */
    bool set_lbd_threshold(float threshold_V);

    /**
     * \brief   Enable the Low Battery Detection
     *
     * \param   enable  boolean, if false then disables detection
     *
     * \return  success
     */
    bool enable_low_battery_detection(bool enable = true);

    /**
     * \brief   Set the length of the preamble
     *
     * \details Set the length of the preamble to send in number of
     *          nibbles. Minimum preamble is of 1 nibble and maximum
     *          is of 255 bytes (510 nibbles).
     *
     * \param   n_nibbles is the number of nibbles the preamble
     *          the preamble should have. 
     *
     * \return  success
     */
    bool set_preamble_length(uint16_t n_nibbles);

    /**
     * \brief   Set the syncword 
     *
     * \return  success
     */
    bool set_syncword(void);

    /**
     * \brief   Set the packet transmit length
     *
     * \details Set the number bytes to send (only data)
     *
     * \param   length  the number of bytes of data to send
     *
     * \return  success
     */
    bool set_packet_transmit_length(uint8_t length);

    /**
     * \brief   Interrupt request handler
     *
     * \return  success
     */
    void irq_handler(void);

    /**
     * \brief   Enables the interrupt service routine
     *
     * \param   enable  boolean, if false then disable the ISRs
     *
     * \return  success
     */
    void isr_enable(bool enable = true);

    /**
     * \brief   Enables the multipacket option for receiving
     *
     * \details In FIFO mode, after a valid packet is received the
     *          device leaves the RX mode automatically. With the
     *          multipacket, this does not happens until the all
     *          the packet are received.
     *
     * \param   enable  boolean, if false then disable the multipacket
     *
     * \return  success
     */
    bool rx_mulit_packet_en(bool enable = true);

    /**
     * \brief   Reads all the status registers of the device and store them
     *
     * \details Read the device status, interrupt status 1 and 2 and ezMAC
     *          status register and store the values in device_status_
     *
     * \return  success
     */
    bool read_status(void);

    /**
     * \brief   Set the cristal oscillator load capacitance
     *
     * \details Change the load capacitance to calibrate the correct frequency
     *
     * \return  success
     */
    bool set_cristal_osci_load_cap(void);

    /**
     * \brief   Set specific bits in a register
     *
     * \param   reg     register's address
     * \param   mask    mask used to set the bits
     *
     * \return  success
     */
    bool set_bit_in_reg(uint8_t reg, uint8_t mask);

    /**
     * \brief   Clear specific bits in a register
     *
     * \param   reg     register's address
     * \param   mask    mask used to clear the bits
     *
     * \return  success
     */
    bool clear_bit_in_reg(uint8_t reg, uint8_t mask);

    /**
     * \brief   Get destination device ID
     *
     * \details If this device is a coordinator then get the
     *          destination id otherwise it gets the coordinator
     *          id.
     *
     * \return  destination id or coordinator id
     */
    uint32_t get_destination_id(void);

    /**
     * \brief   Set TX and RX FIFO's threshold
     *
     * \details Set almost full and almost empty
     *          threshold for TX FIFO and almost
     *          full threshold for RX FIFO.
     *          Used to trigger interrupts.
     *
     * \return  success
     */
    bool set_fifos_threshold(void);

    /**
     * \brief   Configure the device
     *
     * \param   datarate        datarate (0, 1, 2, 3)
     * \param   min_chan        minimum channel
     * \param   max_chan        maximum channel
     * \param   coordinator_id  id of the coordinator to speak to
     * \param   oneway          is one way communication or not
     * \param   ppm_mode        if ppm mode used
     * \param   ppm_only        if ppm only ppm mode used
     *
     * \return  success
     */
    bool config_device(     int datarate,
                            uint8_t min_chan,
                            uint8_t max_chan,
                            uint32_t coordinator_id,
                            bool oneway,
                            bool ppm_mode,
                            bool ppm_only);

    /**
     * \brief   Generate a semi-random number
     *
     * \param   crc     seed
     * \param   data    seed ????
     *
     * \return  semi-random number
     */
    uint8_t update_byte(uint8_t crc, const uint8_t data);

    /**
     * \brief   Initializes the external interrupts
     *
     * \return  success
     */
    bool exti_init(void);

    /**
     * \brief   Set the RX mode
     *
     * \details Prepares the device and enters the RX mode
     *
     * \return  success
     */
    bool set_rx_mode(void);

    /**
     * \brief   ?? implement
     *
     * \return  success
     */
    bool rx_failure(void);

    /**
     * \brief   Process a receiving event
     *
     * \return  success
     */
    bool process_rx(void);

    // Used only for tests
    bool receive(void);
    int transmit(uint8_t* tx_buffer, uint8_t tx_len);
    int prepare_receive(void);
    bool tx_mode_enable(bool enable = true); // not used anymore with Taulabs version but can be re-added if makes code easier
    bool rx_mode_enable(bool enable = true);

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
    static const uint8_t LBD_THRESHOLD      = 0x1A;     // Low Battery Detection Threshold
    static const uint8_t BATT_V_LEVEL       = 0x1B;     // Battery Voltage Level
    static const uint8_t IF_FILTER_BW		= 0x1C;		// IF Filter Bandwidth
    static const uint8_t AFC_LOOP_GS_OVRRD	= 0x1D;		// AFC Loop Gearshift Override
    static const uint8_t AFC_TIMING_CNTL    = 0x1E;     // AFC Timing Control
    static const uint8_t CLK_REC_GS_OVRRD	= 0x1F;		// CLK Recovery Gearshift Override
    static const uint8_t CLK_REC_OVRSMP_RT	= 0x20;		// Clock Recovery Oversampling Ratio
    static const uint8_t CLK_REC_OFST_2     = 0x21;     // Clock Recovery Offset 2
    static const uint8_t CLK_REC_OFST_1     = 0x22;     // Clock Recovery Offset 1
    static const uint8_t CLK_REC_OFST_0     = 0x23;     // Clock Recovery Offset 0
    static const uint8_t CLK_REC_TIM_LPG_1  = 0x24;     // Clock Recovery Timing Loop Gain 1
    static const uint8_t CLK_REC_TIM_LPG_0  = 0x25;     // Clock Recovery Timing Loop Gain 0
    static const uint8_t RSSI_REG           = 0x26;     // Received Signal Strength Indicator
    static const uint8_t RSSI_THRESH_CLR_CH	= 0x27;		// RSSI Threshold for Clear Channel Indicator
    static const uint8_t AFC_LIMITER        = 0x2A;
    static const uint8_t AFC_CORRECTION     = 0x2B;
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
    static const uint8_t RECEIVED_HEADER_3  = 0x47;
    static const uint8_t RECEIVED_HEADER_2  = 0x48;
    static const uint8_t RECEIVED_HEADER_1  = 0x49;
    static const uint8_t RECEIVED_HEADER_0  = 0x4A;
    static const uint8_t RECEIVE_PKT_LEN    = 0x4B;     // Receive Packet Length
    static const uint8_t AGC_OVRRD_1        = 0x69;     // AGC Override 1
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

    static const uint8_t CLEAR_ALL  = 0xFF;
    static const uint8_t SET_ALL    = 0xFF;

    // Operating and Function Control 1 bits
    static const uint8_t OP_CNTL1_MODE_IDLE_READY 		= 0x01;
    static const uint8_t OP_CNTL1_MODE_IDLE_STANDBY 	= 0x00;
    static const uint8_t OP_CNTL1_MODE_WT_EN			= 0x20;
    static const uint8_t OP_CNTL1_MODE_LBD_EN_MSK		= 0x40;
    static const uint8_t OP_CNTL1_MODE_PPL_EN_MSK 		= 0x02;
    static const uint8_t OP_CNTL1_MODE_RX_EN_MSK 		= 0x04;
    static const uint8_t OP_CNTL1_MODE_TX_EN_MSK		= 0x08;
    static const uint8_t OP_CNTL1_SWRESET				= 0x80;

    // Operating and Function Control 2 bits
    static const uint8_t OP_CNTL2_FFCLRTX_MSK   = 0x01;
    static const uint8_t OP_CNTL2_FFCLRRX_MSK   = 0x02;
    static const uint8_t OP_CNTL2_LOW_DC_MSK    = 0x04;
    static const uint8_t OP_CNTL2_RXMPK_MSK     = 0x10;

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
    static const uint8_t MOD_CNTL2_DCLK_GPIO	= 0x40;
    static const uint8_t MOD_CNTL2_DCLK_SDO		= 0x80;
    static const uint8_t MOD_CNTL2_DCLK_NIRQ	= 0xC0;
    static const uint8_t MOD_CNTL2_INV_EN 		= 0x08;

    static const uint8_t MOD_CNTL2_MODTYP_MSK   = 0x03;
    static const uint8_t MOD_CNTL2_DTMOD_MSK    = 0x30;
    static const uint8_t MOD_CNTL2_TRCLK_MSK    = 0xC0;


    // Microcontroller Output Clock bits
    static const uint8_t CPU_OUTPUT_CLK_1MHZ	= 0x06;
    static const uint8_t CPU_OUTPUT_CLK_2MHZ	= 0x05;

    // GPIO 0-1-2 Configuration bits
    static const uint8_t GPIOX_PIN_FUNC_SEL_MSK = 0x1F;
    static const uint8_t GPIOX_DRVG_CAPA_MSK    = 0xC0;
    static const uint8_t GPIOX_CONFIG_TX_STATE	= 0x12;
    static const uint8_t GPIOX_CONFIG_RX_STATE	= 0x15;
    static const uint8_t GPIOX_CLR_CH_ASSESS	= 0x1C; // Clear Channel Assessment
    static const uint8_t GPIOX_DRVG_LVL_3       = 0xC0;

    // Data Access Control bits
    static const uint8_t ACCESS_CRC_EN_MSK      = 0x04;
    static const uint8_t ACCESS_TX_PACK_EN_MSK	= 0x08;
    static const uint8_t ACCESS_RX_PACK_EN_MSK	= 0x80;

    // TX Power bits
    static const uint8_t TX_POWER_LNA_SW		= 0x08;

    // Header Control 1 bits
    static const uint8_t HDR_CNTL1_HDCH_MSK     = 0x0F;
    static const uint8_t HDR_CNTL1_BCEN_MSK     = 0xF0;

    // Header Control 2 bits
    static const uint8_t HDR_CNTL2_PREALEN_MSK  = 0x01;
    static const uint8_t HDR_CNTL2_SYNCLEN_MSK  = 0x06;
    static const uint8_t HDR_CNTL2_FIXPKLEN_MSK = 0x08;
    static const uint8_t HDR_CNTL2_HDLEN_MSK    = 0x30;
    static const uint8_t HDR_CNTL2_NO_HD        = 0x00;
    static const uint8_t HDR_CNTL2_HD_3         = 0x01;
    static const uint8_t HDR_CNTL2_HD_32        = 0x02;
    static const uint8_t HDR_CNTL2_HD_321       = 0x03;
    static const uint8_t HDR_CNTL2_HD_3210      = 0x04;

    static const uint8_t HDR_CNTL2_SYNC_3       = 0x00;
    static const uint8_t HDR_CNTL2_SYNC_32      = 0x01;
    static const uint8_t HDR_CNTL2_SYNC_321     = 0x02;
    static const uint8_t HDR_CNTL2_SYNC_3210    = 0x03;

    // TX FIFO Control register bits
    static const uint8_t TX_FIFO_CNTL1_TXAFTHR_MSK  = 0x3F;
    static const uint8_t TX_FIFO_CNTL2_TXFAETHR_MSK = 0x3F;

    // RX FIFO Control register bits
    static const uint8_t RX_FIFO_CNTL_RXAFTHR_MSK   = 0x3F;

    // Interrupt Enable 1 register bit
    static const uint8_t INT_EN1_ICRCERROR_MSK  = 0x01;
    static const uint8_t INT_EN1_IPKVALID_MSK   = 0x02;
    static const uint8_t INT_EN1_IPKSENT_MSK    = 0x04;
    static const uint8_t INT_EN1_IEXT_MSK       = 0x08;
    static const uint8_t INT_EN1_IRXFFAFULL_MSK = 0x10;
    static const uint8_t INT_EN1_ITXFFAEM_MSK   = 0x20;
    static const uint8_t INT_EN1_ITXFFAFULL_MSK = 0x40;
    static const uint8_t INT_EN1_IFFERR_MSK     = 0x80;

    // Interrupt Enable 2 register bit
    static const uint8_t INT_EN2_IPOR_MSK       = 0x01;
    static const uint8_t INT_EN2_ICHIPRDY_MSK   = 0x02;
    static const uint8_t INT_EN2_ILBD_MSK       = 0x04;
    static const uint8_t INT_EN2_IWUT_MSK       = 0x08;
    static const uint8_t INT_EN2_IRSSI_MSK      = 0x10;
    static const uint8_t INT_EN2_IPREAINVAL_MSK = 0x20;
    static const uint8_t INT_EN2_IPREAVAL_MSK   = 0x40;
    static const uint8_t INT_EN2_ISWDET_MSK     = 0x80;

    // TX Power register bit
    static const uint8_t TX_POW_TXPOW_MSK       = 0x07;

    // Preamble Detection Control register bits
    static const uint8_t PREAM_DET_CNTL_RSSI_OFF_MSK    = 0x07;
    static const uint8_t PREAM_DET_CNTL_PREATH_MSK      = 0xF8;

    // Low Battery Detection Threshold register bits
    static const uint8_t LBD_THRESH_LBDT_MSK    = 0x1F;

    // public only for tests purpose

    Gpio&   nirq_;  ///< Interrupt GPIO (active low)
    uint32_t counter_var; ///< Received message length in bytes
    bool    rx_success_;
    uint8_t*        rx_buffer_;         ///< Reception buffer
    



 private:
 	Spi& 	spi_;           ///< SPI peripheral
 	Gpio&	nss_;           ///< Slave Select GPIO
    // Gpio&   nirq_;       ///< Interrupt GPIO (active low)
    uint8_t rx_len_;        ///< Received message length in bytes
    // bool    rx_success_;
    uint8_t*        tx_buffer_;         ///< Transmission buffer
    // uint8_t*        rx_buffer_;      ///< Reception buffer
    Led_gpio&   led_irq_;
    device_status_t     device_status_; ///< device status
    bool     coordinator_;              ///< true if coordinator
    uint32_t coordinator_id_;           ///< coordinator ID
    uint32_t device_id_;                ///< device ID
    bool ppm_send_mode_;                ///< true if ppm send mode enabled
    bool ppm_recv_mode_;                ///< true if ppm recv mode enabled
    bool ppm_only_mode_;                ///< true if ppm mode only used
    bool one_way_link_;                 ///< true if one way communication
    int datarate_;                      ///< datarate (0 = 9.6kbps, TODO, make this more explicit by using enum?)
    uint8_t packet_time_;               ///< ??
    uint8_t channels_[250];             ///< list of channels for FHSS
    uint8_t max_packet_len_;            ///< Max size of a packet
    uint8_t destination_id_;            ///< ID of the destination device
    float frequency_step_size_;         ///< frequency step size
    int8_t afc_correction_hz_;          ///< afc correction in herz
    uint8_t rssi_dBm_;                  ///< RSSI in dBm
    uint16_t rx_buffer_wr_;             ///< ??
    uint32_t rx_destination_id_;        ///< RX destination ID
    transaction_state_t trans_state_;   ///< transaction state

    /**
     * \brief       Callback function to be called after an interrupt
     *
     * \details     By default NULL, can be modified via the 'attach' method
     */
    serial_interrupt_callback_t irq_callback;

 	/**
     * \brief   Select Slave
     */
    void select_slave(void);

    /**
     * \brief   Unselect Slave
     */
    void unselect_slave(void);
 };