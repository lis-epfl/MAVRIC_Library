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
 * \file    rfm22b.cpp
 *
 * \author  MAV'RIC Team
 * \author  Jean-Francois Burnier
 *
 * \brief   This file is the driver for the rfm22b, a RF transceiver
 *
 ******************************************************************************/
 #include <cstddef>
 #include "drivers/rfm22b.hpp"

 #include "hal/common/time_keeper.hpp"

#include "util/string_util.hpp"


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Rfm22b::Rfm22b(Spi& spi, Gpio& nss_gpio, Serial& serial)://, const conf_t config):
spi_(spi),
nss_(nss_gpio),
serial_(serial),
console(serial)
{}

// bool Rfm22b::init()
// {
// 	bool success = true;

// 	// Initializing Slave Select GPIO
//     success &= nss_.init();
//     unselect_slave();

//     // Reading Device Type
// 	uint8_t device_type_answer = 0x00;
// 	success &= read_reg(DEVICE_TYPE_REG, &device_type_answer);
// 	success &= device_type_answer == DEVICE_TYPE ? true : false;

// 	// Reading Device Version
// 	uint8_t device_version_answer = 0x00;
// 	success &= read_reg(DEVICE_VERSION_REG, &device_version_answer);
// 	success &= device_version_answer == DEVICE_VERSION ? true : false;

// 	// Reading Interrupt Enable
// 	uint8_t interen1 = 0x00;
// 	uint8_t interen2 = 0x00;
// 	success &= read_reg(INTERRUPT_EN_1, &interen1);
// 	success &= read_reg(INTERRUPT_EN_2, &interen2);

// 	// Reading Operating Function Control
// 	uint8_t op_func_cntl_1 = 0x00;
// 	uint8_t op_func_cntl_2 = 0x00;
// 	success &= read_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);
// 	success &= read_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

//  //    // Reset rfm22b
//  //    success &= reset();

//  //    for (uint8_t i = 0; i < 50; ++i)
//  //    {
//  //    	uint8_t read_buf1 = 0;
//  //    	uint8_t read_buf2[2] = {0};
//  //    	success &= read_reg(INTERRUPT_STAT_1, read_buf2, 2);

//  //    	success &= read_reg(DEVICE_STATUS, &read_buf1);
//  //    	success &= read_reg(EZMAC_STATUS, &read_buf1);

//  //    	time_keeper_delay_ms(1);
//  //    }

//  //    uint8_t read_buf1 = 0;
// 	// uint8_t read_buf2[2] = {0};
// 	// success &= read_reg(INTERRUPT_STAT_1, read_buf2, 2);

// 	// success &= read_reg(DEVICE_STATUS, &read_buf1);
// 	// success &= read_reg(EZMAC_STATUS, &read_buf1);

// 	// time_keeper_delay_ms(1);

//  //    // Disable interrupts
//  //    uint8_t dis_int = 0x00;
//  //    success &= write_reg(INTERRUPT_EN_1, &dis_int);
//  //    success &= write_reg(INTERRUPT_EN_2, &dis_int);

//  //    // Test to see if device is responding
// 	// uint8_t device_type_answer = 0x00;
// 	// success &= read_reg(DEVICE_TYPE_REG, &device_type_answer);
// 	// success &= device_type_answer == DEVICE_TYPE ? true : false;

// 	// if (!success)
// 	// {
// 	// 	return success;
// 	// }

// 	// time_keeper_delay_ms(1);

// 	// // Set Cristal Oscillator to get desired frequency
// 	// uint8_t osc_load_cap = 0x7F;
// 	// success &= write_reg(OSC_LOAD_CAP_REG, &osc_load_cap);

// 	// // Disable low duty cycle mode
// 	// uint8_t duty_cycle_mode = 0x00;
// 	// success &= write_reg(OP_FUNC_CNTL_2, &duty_cycle_mode);

// 	// // Set output clock to 1MHz
// 	// uint8_t output_clock = CPU_OUTPUT_CLK_1MHZ;
// 	// success &= write_reg(CPU_OUTPUT_CLK, &output_clock);

// 	// // Set Ready Mode
// 	// uint8_t set_mode = OP_CNTL1_MODE_IDLE_READY;
// 	// success &= write_reg(OP_FUNC_CNTL_1, &set_mode);

// 	// // Configure I/O ports
// 	// uint8_t io_port_config 	= 0x00; // Default config
// 	// uint8_t gpio0_config 	= 0xC0 | GPIOX_CONFIG_TX_STATE;
// 	// uint8_t gpio1_config 	= 0xC0 | GPIOX_CONFIG_RX_STATE;
// 	// uint8_t gpio2_config	= 0xC0 | GPIOX_CLR_CH_ASSESS;
// 	// success &= write_reg(IO_PORT_CONFIG, &io_port_config);
// 	// success &= write_reg(GPIO0_CONFIG_REG, &gpio0_config);
// 	// success &= write_reg(GPIO1_CONFIG_REG, &gpio1_config);
// 	// success &= write_reg(GPIO2_CONFIG_REG, &gpio2_config);

// 	// time_keeper_delay_ms(1);

// 	// // Set Data Source and Modulation Mode
// 	// uint8_t fd_msb = 0;
// 	// success &= read_reg(MOD_MODE_CNTL_2, &fd_msb);
// 	// fd_msb  &= MOD_CNTL2_FD_MSB;
// 	// uint8_t data_source = MOD_CNTL2_FIFO | MOD_CNTL2_GFSK | fd_msb;
// 	// success &= write_reg(MOD_MODE_CNTL_2, &data_source);

// 	// //Setups to read internal temperature sensor
// 	// temp_sens_config();

// 	// // Enabling TX/RX packet handling
// 	// uint8_t data_access = ACCESS_TX_PACK_EN | ACCESS_RX_PACK_EN;
// 	// success &= write_reg(DATA_ACCESS_CNTL, &data_access);


// 	// // Set Preamble Length
// 	// uint8_t preamble_length = 0x0C;
// 	// success &= write_reg(PREAMBLE_LEN_REG, &preamble_length);

// 	// // Set Preamble Detection Length
// 	// uint8_t preamble_detection = 6 << 3;
// 	// success &= write_reg(PREAMBLE_DET_CNTL, &preamble_detection);

// 	// // Set header
// 	// uint8_t header_control = 0xFF;
// 	// success &= write_reg(HEADER_CNTL_1, &header_control);

// 	// // Header enable
// 	// uint8_t header_enable = 0xFF;
// 	// success &= write_reg(0x46, &header_enable);
// 	// success &= write_reg(0x45, &header_enable);
// 	// success &= write_reg(0x44, &header_enable);
// 	// success &= write_reg(0x43, &header_enable);

// 	// // Header Length Syncword Length
// 	// uint8_t hdlen_synclen = 0x40 | 0x06;
// 	// success &= write_reg(HEADER_CNTL_2, &hdlen_synclen);

// 	// time_keeper_delay_ms(1);

// 	// // Set Syncword
// 	// uint8_t sync_byte_1 = 0x2D;
// 	// uint8_t sync_byte_2 = 0xD4;
// 	// uint8_t sync_byte_3 = 0x4B;
// 	// uint8_t sync_byte_4 = 0x59;
// 	// success &= write_reg(SYNC_WORD_3, &sync_byte_1);
// 	// success &= write_reg(SYNC_WORD_2, &sync_byte_2);
// 	// success &= write_reg(SYNC_WORD_1, &sync_byte_3);
// 	// success &= write_reg(SYNC_WORD_0, &sync_byte_4);

// 	// // Set FIFO threshold
// 	// uint8_t tx_fifo_thresh_h = 62;
// 	// uint8_t tx_fifo_thresh_l = 32;
// 	// uint8_t rx_fifo_thresh_h = 32;
// 	// success &= write_reg(TX_FIFO_CNTL_1, &tx_fifo_thresh_h);
// 	// success &= write_reg(TX_FIFO_CNTL_2, &tx_fifo_thresh_l);
// 	// success &= write_reg(RX_FIFO_CNTL  , &rx_fifo_thresh_h);

// 	// // Set Cristal Oscillator to get desired frequency
// 	// osc_load_cap = 0x7F;
// 	// success &= write_reg(OSC_LOAD_CAP_REG, &osc_load_cap);

// 	// time_keeper_delay_ms(1);

// 	// // Set Nominal Carrier Frequency
// 	// set_normal_carrier_freq();

// 	// // Set Data Rate
// 	// set_datarate();

// 	// // modulation mode control 1
// 	// uint8_t mode_1 = 0x2C | MOD_CNTL1_WHITE_EN;
// 	// success &= write_reg(MOD_MODE_CNTL_1, &mode_1);

// 	// // modulation mode control 2
// 	// uint8_t mode_2 = 0x23;
// 	// success &= write_reg(MOD_MODE_CNTL_2, &mode_2);

// 	// // frequency deviation
// 	// uint8_t frequency_deviation = 0x30;
// 	// success &= write_reg(FREQ_DEVIATION, &frequency_deviation);

// 	// // ook counter val 1
// 	// uint8_t ook_counter_val_1 = 0x00;
// 	// success &= write_reg(OOK_COUNTER_VAL_1, &ook_counter_val_1);

// 	// // ook counter val 2
// 	// uint8_t ook_counter_val_2 = 0x00;
// 	// success &= write_reg(OOK_COUNTER_VAL_2, &ook_counter_val_2);

// 	// // Set Mode
// 	// set_mode = OP_CNTL1_MODE_TX_ON;
// 	// success &= write_reg(OP_FUNC_CNTL_1, &set_mode);

// 	return success;
// }

bool Rfm22b::init()
{
	bool success = true;

	// Initializing Slave Select GPIO
    success &= nss_.init();
    unselect_slave();

    // Reseting device
    success &= reset();

    // Reading Device Type
	uint8_t device_type_answer = 0x00;
	success &= read_reg(DEVICE_TYPE_REG, &device_type_answer);
	success &= device_type_answer == DEVICE_TYPE ? true : false;

	// Reading Device Version
	uint8_t device_version_answer = 0x00;
	success &= read_reg(DEVICE_VERSION_REG, &device_version_answer);
	success &= device_version_answer == DEVICE_VERSION ? true : false;

    // Set carrier frequency
    success &= set_carrier_frequency(433E6);//869.5E6);

    // Set modulation Type
    success &= set_modulation_type();

    // Set modulation data source
    success &= set_modulation_data_source();

    // Set data clock configuration
    success &= set_data_clock_configuration();

    // Set TX power
    success &= set_transmission_power(20);//20;

    // Configure GPIOS
    success &= set_gpio_function();

    success &= interrput_enable(0xFF,0xFF);//0x77,0xF0);

    // Preamble Configuration
    success &= set_preamble_length(10);//40);
    success &= set_preamble_detection(5);//5);

    // Syncword Configuration
    //
    uint8_t syncword[4] = {0x2D, 0x82, 0xEA, 0x1F};
    success &= set_syncword_length(0x03);//0);
    success &= set_syncword(syncword);

    // Header Configuration
    //
    uint8_t transmit_header[4] = {0xAB, 0xBC, 0xCD, 0xDE};

    success &= set_transmit_header(transmit_header);
    success &= set_check_header(transmit_header);
    success &= set_header_length(0x01);
    success &= set_header_check();
    success &= header_enable();

    // Low Battery Detection Configuration
    success &= enable_low_battery_detection();
    success &= set_lbd_threshold(0x1F);

    // // RSSI Configuration
    // success &= set_rssi_offset(10);
    // success &= set_rssi_threshold(0x60);//0x1E);

	return success;
}

bool Rfm22b::attach(serial_interrupt_callback_t func)
{
	bool ret = true;

	return ret;
}

void Rfm22b::flush(void)
{
	return;
}

bool Rfm22b::read_reg(uint8_t reg, uint8_t* in_data, uint32_t nbytes)
{
    bool ret = true;
    reg &= READ_FLAG;

    select_slave();

    // Write register
    ret &= spi_.transfer(&reg, NULL, 1);

    // Read data from register
    ret &= spi_.transfer(NULL, in_data, nbytes);

    unselect_slave();

    return ret;
}

bool Rfm22b::write_reg(uint8_t reg, uint8_t* out_data, uint32_t nbytes)
{
    bool ret = true;
    reg |= WRITE_FLAG;

    select_slave();

    // Write register
    ret &= spi_.transfer(&reg, NULL, 1);

    // Write data to register
    ret &= spi_.transfer(out_data, NULL, nbytes);

    unselect_slave();


    return ret;
}

uint32_t Rfm22b::readable(void)
{
	return 0;
}

uint32_t Rfm22b::writeable(void)
{
	return 0;
}

bool Rfm22b::read(uint8_t* bytes, const uint32_t size)
{
	bool ret = true;
	// int success = 0;

	// uint8_t message[64] 	= {0};
	// uint8_t acknowledge[12] = "acknowledge";
	// uint8_t rx_len 			= 0;

	// success = receive(message, &rx_len);
	// if (success != 1)
	// {
	// 	return false;
	// }

	// success = transmit(acknowledge, 11);
	// if (success != 1)
	// {
	// 	return false;
	// }

	return true;
}

int Rfm22b::read_test(uint8_t* message, uint8_t* rx_len)
{
	bool ret = true;
	int success = 1;

	// uint8_t message[64] 	= {0};
	uint8_t acknowledge[12] = "acknowledge";
	// uint8_t rx_len 			= 0;

	success = receive(message, rx_len);
	if (success != 1)
	{
		return success;
	}

	success = transmit(acknowledge, 11);
	if (success != 1)
	{
		return success;
	}

	return success;
}

bool Rfm22b::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = true;
	// int success = 0;

	// uint8_t message[64] 	= "Hello World!";
	// uint8_t acknowledge[12] = "acknowledge";
	// uint8_t ack_msg[12] 	= {0};
	// uint8_t rx_len 			= 0;

	// success = transmit(message, 12);
	// if (success != 1)
	// {
	// 	return false;
	// }

	// success = receive(ack_msg, &rx_len);
	// if (success != 1)
	// {
	// 	return false;
	// }

	// for (int i = 0; i < 12; i++)
	// {
	// 	if (acknowledge[i] != ack_msg[i])
	// 	{
	// 		success = -3;
	// 		break;
	// 	}
	// }

	// if (success == 1)
	// {
	// 	return true;
	// }
	// else
	// {
	// 	return false;
	// }

	return ret;

}

int Rfm22b::write_test(uint8_t* message, uint8_t tx_len)
{
	bool ret = true;
	int success = 1;

	// uint8_t message[64] 	= "Hello";
	uint8_t acknowledge[12] = "acknowledge";
	uint8_t ack_msg[12] 	= {0};
	uint8_t rx_len 			= 0;

	success = transmit(message, tx_len);
	if (success != 1)
	{
		return success;
	}

    // const char* newline = "\r\n";
    // console.write(' ');
    // time_keeper_delay_ms(5);
    // serial_.write((const uint8_t*)newline, sizeof(newline));
    // time_keeper_delay_ms(5);


	success = receive(ack_msg, &rx_len);
	if (success != 1)
	{
		return success;
	}

	for (int i = 0; i < 12; i++)
	{
		if (acknowledge[i] != ack_msg[i])
		{
			success = -5;
			break;
		}
	}

	return success;
}

void Rfm22b::select_slave(void)
{
    nss_.set_low();
    return;
}

void Rfm22b::unselect_slave(void)
{
    nss_.set_high();
    return;
}

bool Rfm22b::reset(void)
{
    bool ret = true;

    uint8_t reset_command = OP_CNTL1_SWRESET |
    						OP_CNTL1_MODE_IDLE_READY;

    // Write reset
    ret &= write_reg(OP_FUNC_CNTL_1, &reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}

// bool Rfm22b::temp_sens_config(void)
// {
// 	bool ret = true;

// 	// ADC used to sample the temperature sensor
// 	uint8_t adc_config = ADC_SOURCE_SEL_TEMP_SENSOR;
// 	ret &= write_reg(ADC_CONFIG, &adc_config);

// 	// ADC offset
// 	uint8_t adc_offset = 0x00;
// 	ret &= write_reg(ADC_SENS_AMP_OFST, &adc_offset);

// 	// Temperature sensor calibration
// 	uint8_t temp_sens_calib = TEMP_SENS_CALIB_0;
// 	ret &= write_reg(TEMP_SENS_CNTL, &temp_sens_calib);

// 	// Temperature sensor offset
// 	uint8_t temp_offset = 0x00;
// 	ret &= write_reg(TEMP_VAL_OFST, &temp_offset);

// 	// Start an ADC conversion
// 	adc_config |= ADC_START;
// 	ret &= write_reg(ADC_CONFIG, &adc_config);

// 	// Set the RSSI threshold interrupt to about -90dBm
// 	uint8_t rssi_thresh = (-90 + 122) * 2;
// 	ret &= write_reg(RSSI_THRESH_CLR_CH, &rssi_thresh);

// 	time_keeper_delay_ms(1);

// 	return ret;
// }

// bool Rfm22b::set_normal_carrier_freq(void)
// {
// 	bool ret = true;

// 	// holds the hbsel (1 or 2)
// 	uint8_t hbsel;
// 	uint32_t frequency_hz = 430000000; //Hz

// 	if (frequency_hz < 480000000) {
// 		hbsel = 0;
// 	} else {
// 		hbsel = 1;
// 	}
// 	float freq_mhz = (float)(frequency_hz) / 1000000.0f;
// 	float xtal_freq_khz = 30000.0f;
// 	float sfreq = freq_mhz / (10.0f * (xtal_freq_khz / 30000.0f) * (1 + hbsel));
// 	uint32_t fb = (uint32_t) sfreq - 24 + (64 + 32 * hbsel);
// 	uint32_t fc = (uint32_t) ((sfreq - (uint32_t) sfreq) * 64000.0f);
// 	uint8_t fch = (fc >> 8) & 0xff;
// 	uint8_t fcl = fc & 0xff;

// 	// Set the frequency hopping step size, the step size is
// 	// 10MHz / 250 channels = 40khz, and the step size is
// 	// specified in 10khz increments.
// 	uint8_t freq_hop_step_size = 4;
// 	ret &= write_reg(FREQ_HOP_STEP_SIZE, &freq_hop_step_size);

// 	// Frequency Hopping Channel
// 	uint8_t freq_hop_ch_sel = 0x00;
// 	ret &= write_reg(FREQ_HOP_CH_SEL, &freq_hop_ch_sel);

// 	// No frequency Offset
// 	uint8_t frequency_offset = 0x00;
// 	ret &= write_reg(FREQ_OFFSET_1, &frequency_offset);
// 	ret &= write_reg(FREQ_OFFSET_2, &frequency_offset);

// 	// Set the carrier frequency
// 	uint8_t freq_band_sel = fb & 0xFF;
// 	uint8_t nrml_carr_freq_1 = fch;
// 	uint8_t nrml_carr_freq_0 = fcl;
// 	ret &= write_reg(FREQ_BAND_SEL, &freq_band_sel);
// 	ret &= write_reg(NRML_CARR_FREQ_1, &nrml_carr_freq_1);
// 	ret &= write_reg(NRML_CARR_FREQ_0, &nrml_carr_freq_0);

// 	return ret;
// }

// bool Rfm22b::set_datarate(void)
// {
// 	bool ret = true;

// 	// if filter bandwidth
// 	uint8_t if_filter_bw = 0x01;
// 	ret &= write_reg(0x1C, &if_filter_bw);

// 	// afc loop gearshift override
// 	uint8_t afc_loop_g_o = 0x40;
// 	ret &= write_reg(0x1D, &afc_loop_g_o);

// 	// afc timing control
// 	uint8_t afc_timing_control = 0x0A;
// 	ret &= write_reg(0x1E, &afc_timing_control);

// 	// Clock Recovery Gearshift Override
// 	uint8_t clk_rec_g_o = 0x03;
// 	ret &= write_reg(0x1F, &clk_rec_g_o);

// 	// Clock Recovery Oversampling Ratio
// 	uint8_t clk_rec_ovrsamp_r = 0xA1;
// 	ret &= write_reg(0x20, &clk_rec_ovrsamp_r);

// 	// Clock Recorvery Offset 2
// 	uint8_t clk_rec_offset_2 = 0x20;
// 	ret &= write_reg(0x21, &clk_rec_offset_2);

// 	// Clock Recorvery Offset 1
// 	uint8_t clk_rec_offset_1 = 0x4E;
// 	ret &= write_reg(0x22, &clk_rec_offset_1);

// 	// Clock Recorvery Offset 0
// 	uint8_t clk_rec_offset_0 = 0xA5;
// 	ret &= write_reg(0x23, &clk_rec_offset_0);

// 	// Clock Recovery Timing Loop Gain 1
// 	uint8_t clk_rec_t_loop_gain_1 = 0x00;
// 	ret &= write_reg(0x24, &clk_rec_t_loop_gain_1);

// 	// Clock Recovery Timing Loop Gain 0
// 	uint8_t clk_rec_t_loop_gain_0 = 0x34;
// 	ret &= write_reg(0x25, &clk_rec_t_loop_gain_0);

// 	// agc override 1
// 	uint8_t agc_override_1 = 0x60;
// 	ret &= write_reg(0x69, &agc_override_1);

// 	// afc limiter
// 	uint8_t afc_limiter = 0x1E;
// 	ret &= write_reg(0x2A, &afc_limiter);

// 	// tx data rate 1
// 	uint8_t tx_data_rate_1 = 0x4E;
// 	ret &= write_reg(0x6E, &tx_data_rate_1);

// 	// tx data rate 0
// 	uint8_t tx_data_rate_0 = 0xA5;
// 	ret &= write_reg(0x6F, &tx_data_rate_0);

// 	return ret;
// }

// bool Rfm22b::rfm22b_init(void)
// {
// 	bool ret = true;

// 	// Disable interrupt
// 	uint8_t dis_int = 0x00;
// 	ret &= write_reg(INTERRUPT_EN_2, &dis_int);

// 	// Set Ready Mode
// 	uint8_t set_mode = OP_CNTL1_MODE_IDLE_READY;
// 	ret &= write_reg(OP_FUNC_CNTL_1, &set_mode);

// 	// Set Cristal Oscillator to get desired frequency
// 	uint8_t osc_load_cap = 0x7F;
// 	ret &= write_reg(OSC_LOAD_CAP_REG, &osc_load_cap);

// 	// Set output clock to 2MHz
// 	uint8_t output_clock = CPU_OUTPUT_CLK_2MHZ;
// 	ret &= write_reg(CPU_OUTPUT_CLK, &output_clock);

// 	// Configure I/O ports
// 	uint8_t gpio0_config 	= 0xF4;	// GPIO0 is for RX data output
// 	uint8_t gpio1_config 	= 0xEF;	// GPIO1 is TX/RX data CLK output
// 	uint8_t gpio2_config	= 0x00;	// GPIO2 for MCLK output
// 	uint8_t io_port_config 	= 0x00;	// Default config
// 	ret &= write_reg(GPIO0_CONFIG_REG, &gpio0_config);
// 	ret &= write_reg(GPIO1_CONFIG_REG, &gpio1_config);
// 	ret &= write_reg(GPIO2_CONFIG_REG, &gpio2_config);
// 	ret &= write_reg(IO_PORT_CONFIG, &io_port_config);

// 	// No ADC
// 	uint8_t adc_config = 0x70;
// 	ret &= write_reg(ADC_CONFIG, &adc_config);

// 	uint8_t adc_offset = 0x00;
// 	ret &= write_reg(ADC_SENS_AMP_OFST, &adc_offset);

// 	// No Temperature Sensor
// 	uint8_t temp_sens_calib = 0x00;
// 	ret &= write_reg(TEMP_SENS_CNTL, &temp_sens_calib);

// 	// No Temperature Sensor
// 	uint8_t temp_offset = 0x00;
// 	ret &= write_reg(TEMP_VAL_OFST, &temp_offset);

// 	// No Manchester, no whitening, data rate < 30Kbps
// 	uint8_t	mod_mode_cntl_1 = 0x20;
// 	ret &= write_reg(MOD_MODE_CNTL_1, &mod_mode_cntl_1);

// 	// IF filter bandwidth
// 	uint8_t if_filter_bw = 0x1D;
// 	ret &= write_reg(IF_FILTER_BW, &if_filter_bw);

// 	// AFC Loop Gearshift Override
// 	uint8_t afc_loop_gs_ovrrd = 0x40;
// 	ret &= write_reg(AFC_LOOP_GS_OVRRD, &afc_loop_gs_ovrrd);

// 	// Clock Recovery
// 	uint8_t clk_recovery[6] = {0xA1, 0x20, 0x4E, 0xA5, 0x00, 0x0A};
// 	ret &= write_reg(CLK_REC_OVRSMP_RT, clk_recovery, 6);

// 	// OOK Counter Value
// 	uint8_t ook_cnt_val[2] = {0x00, 0x00};
// 	ret &= write_reg(OOK_COUNTER_VAL_1, ook_cnt_val, 2);

// 	// Slice Peak Hold
// 	uint8_t slice_peak_hold = 0x00;
// 	ret &= write_reg(SLICER_PEAK_HOLD, &slice_peak_hold);

// 	// TX Data Rate
// 	uint8_t tx_data_rate[2] = {0x27, 0x52};
// 	ret &= write_reg(TX_DATA_RATE_1, tx_data_rate, 2);

// 	// Data Access Control
// 	uint8_t data_access_cntl = 0x8C;
// 	ret &= write_reg(DATA_ACCESS_CNTL, &data_access_cntl);

// 	// Header Control
// 	uint8_t header_cntl[2] = {0xFF, 0x42};
// 	ret &= write_reg(HEADER_CNTL_1, header_cntl, 2);

// 	// Preamble Length
// 	uint8_t preamble_len_reg = 64; // 32 byte, 64 nibble
// 	ret &= write_reg(PREAMBLE_LEN_REG, &preamble_len_reg);

// 	// Preamble Detection Control
// 	uint8_t preamble_det_cntl = 0x20;
// 	ret &= write_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

// 	// Sync word
// 	uint8_t syncword[4] = {0x2D, 0xD4, 0x00, 0x00};
// 	ret &= write_reg(SYNC_WORD_3, syncword, 4);

// 	// Transmit Header
// 	uint8_t transmit_header[4] = {0xab,0xbc,0xcd,0xde};
// 	ret &= write_reg(TRANSMIT_HEADER_3, transmit_header, 4);

// 	// Transmit Packet Length
// 	uint8_t transmit_pkt_len = 17;
// 	ret &= write_reg(TRANSMIT_PKT_LEN, &transmit_pkt_len);

// 	// Check Header
// 	uint8_t check_header[4] = {0xab,0xbc,0xcd,0xde};
// 	ret &= write_reg(CHECK_HEADER_3, check_header, 4);

// 	// Header Enable
// 	uint8_t header_en[4] = {0xFF,0xFF,0xFF,0xFF};
// 	ret &= write_reg(HEADER_EN_3, header_en, 4);

// 	// register 0x56!!

// 	// TX Power
// 	uint8_t tx_power = 0x07;
// 	ret &= write_reg(TX_POWER, &tx_power);

// 	// No frequency hopping
// 	uint8_t freq_hop[2] = {0x00, 0x00};
// 	ret &= write_reg(FREQ_HOP_CH_SEL, freq_hop, 2);

// 	// Modulation Mode
// 	uint8_t mod_mode = 0x22;
// 	ret &= write_reg(MOD_MODE_CNTL_2, &mod_mode);

// 	// Frequency Deviation
// 	uint8_t freq_dev = 0x48;
// 	ret &= write_reg(FREQ_DEVIATION, &freq_dev);

// 	// Frequency Offset
// 	uint8_t freq_offset[2] = {0x00, 0x00};
// 	ret &= write_reg(FREQ_OFFSET_1, freq_offset, 2);

// 	// Frequency Band Selection
// 	uint8_t freq_band_sel = 0x53;
// 	ret &= write_reg(FREQ_BAND_SEL, &freq_band_sel);

// 	// Normal Carrier Frequency
// 	uint8_t nrml_carr_freq[2] = {0x64, 0x00};
// 	ret &= write_reg(NRML_CARR_FREQ_1, nrml_carr_freq, 2);

// 	// registers 0x5A, 0x58, 0x59 !!!

// 	// registers 0x6A, 0x68 !!!

// 	// Clk Recovery Gearshift Override
// 	uint8_t clk_rec_gs_ovrrd = 0x03;
// 	ret &= write_reg(CLK_REC_GS_OVRRD, &clk_rec_gs_ovrrd);

// 	return ret;
// }

// bool Rfm22b::to_tx_mode(void)
// {
// 	bool ret = true;

// 	unsigned char i;
	

// 	// Set Ready Mode
// 	uint8_t set_mode = OP_CNTL1_MODE_IDLE_READY;
// 	ret &= write_reg(OP_FUNC_CNTL_1, &set_mode);

// 	cbi(PORTD, RXANT);
// 	sbi(PORTD, TXANT);

// 	time_keeper_delay_ms(50);
	
// 	uint8_t fifo_reset = OP_CNTL2_FIFOS_RESET;
// 	ret &= write_reg(OP_FUNC_CNTL_2, &fifo_reset);

	
// 	write(0x08, 0x03);	// FIFO reset
// 	write(0x08, 0x00);	// Clear FIFO
	
// 	write(0x34, 64);	// preamble = 64nibble
// 	write(0x3E, 17);	// packet length = 17bytes
// 	for (i=0; i<17; i++)
// 	{
// 		write(0x7F, tx_buf[i]);	// send payload to the FIFO
// 	}

// 	write(0x05, 0x04);	// enable packet sent interrupt
// 	i = read(0x03);		// Read Interrupt status1 register
// 	i = read(0x04);
	
// 	write(0x07, 9);	// Start TX
	
// 	while ((PIND & (1<<NIRQ)) != 0)
// 		; 	// need to check interrupt here
	
// 	write(0x07, 0x01);	// to ready mode
	
// 	cbi(PORTD, RXANT);	// disable all interrupts
// 	cbi(PORTD, TXANT);

// 	return ret;
// }

bool Rfm22b::set_carrier_frequency(unsigned int frequency)
{
	bool ret = true;

	// If frequency is outside specified range than return false
	if (frequency < 240E6 || frequency > 960E6)
	{
		return false;
	}

	uint8_t fbs = 0;

	// Read frequency band select register
	ret &= read_reg(FREQ_BAND_SEL, &fbs);

	// Clearing fb, hbsel and sbsel bits
	fbs &= 0x80;

	// For high frequencies, hbsel should be 1
	uint8_t hbsel = (frequency >= 480E6);

	// Frequency Band
	uint8_t fb = frequency/10E6/(hbsel+1)-24;

	fbs |= (1 << 6) | (hbsel << 5) | fb;

	// Fractional part of the frequency
	// Warning: Assuming Freq Offset and Freq Hopping are 0
	uint16_t fc = (frequency/(10E6F*(hbsel+1)) - fb - 24) * 64000;

	// Convert fc into two 8 bits variables
	uint8_t ncf1 = (fc >> 8);
	uint8_t ncf0 = fc & 0xFF;



	ret &= write_reg(FREQ_BAND_SEL, &fbs);
	ret &= write_reg(NRML_CARR_FREQ_1, &ncf1);
	ret &= write_reg(NRML_CARR_FREQ_0, &ncf0);

	return ret;
}

bool Rfm22b::set_modulation_type(void)
{
	bool ret = true;

	uint8_t mmc2 = 0;

	// Read MMC2 reg
	ret &= read_reg(MOD_MODE_CNTL_2, &mmc2);

	// Clear the modtyp bits
	mmc2 &= ~0x03;

	// Set the desired modulation
	mmc2 |= MOD_CNTL2_GFSK;

	// Writing new value to register
	ret &= write_reg(MOD_MODE_CNTL_2, &mmc2);

	return ret;
}

bool Rfm22b::set_modulation_data_source(void)
{
	bool ret = true;

	uint8_t mmc2 = 0;

	// Read MMC2 reg
	ret &= read_reg(MOD_MODE_CNTL_2, &mmc2);

	// Clear the dtmod bits
	mmc2 &= ~(0x03<<4);

	// Set the desired data source
	mmc2 |= MOD_CNTL2_FIFO;

	// Write new value to register
	ret &= write_reg(MOD_MODE_CNTL_2, &mmc2);

	return ret;
}

bool Rfm22b::set_data_clock_configuration(void)
{
	bool ret = true;

	uint8_t mmc2 = 0;

	// Read MMC2 reg
	ret &= read_reg(MOD_MODE_CNTL_2, &mmc2);
	
	// Clear the trclk bits
	mmc2 &= ~(0x03<<6);
	
	// Set the desired data source
	mmc2 |= 0x00 << 6;

	// Write new value to register
	ret &= write_reg(MOD_MODE_CNTL_2, &mmc2);

	return ret;
}

bool Rfm22b::set_transmission_power(uint8_t power)
{
	bool ret = true;

	if (power > 20)
	{
		power = 20;
	}

	uint8_t tx_power_reg = 0;
	
	// Read TX power register
	ret &= read_reg(TX_POWER, &tx_power_reg);

	// Clear txpow bits
	tx_power_reg &= ~(0x07);

	uint8_t tx_pow = (power + 1) / 3;

	// Set the desired tx power
	tx_power_reg |= tx_pow;

	// Write new value to register
	ret &= write_reg(TX_POWER, &tx_power_reg);

	return ret;
}

bool Rfm22b::set_gpio_function(void)
{
	bool ret = true;

	uint8_t gpio0 = 0;
	uint8_t gpio1 = 0;

	// Read GPIOx config registers
	ret &= read_reg(GPIO0_CONFIG_REG, &gpio0);
	ret &= read_reg(GPIO1_CONFIG_REG, &gpio1);

	// Clear GPIOx bits
	gpio0 &= ~((1<<5)-1);
	gpio1 &= ~((1<<5)-1);

	// Set the GPIOx bits
	gpio0 |= 0x12;
	gpio1 |= 0x15;

	// Write new value to register
	ret &= write_reg(GPIO0_CONFIG_REG, &gpio0);
	ret &= write_reg(GPIO1_CONFIG_REG, &gpio1);

	return ret;
}

bool Rfm22b::set_transmit_header(uint8_t* txhd)
{
	return write_reg(TRANSMIT_HEADER_3, txhd, 4);
}

bool Rfm22b::send(uint8_t *data, int length)
{
	bool ret = true;
	uint32_t timeout = 0;

	// Clear TX FIFO
	clear_tx_fifo();

	// Truncate data if its too long
	if (length > 64) {
		length = 64;
	}

	ret &= set_packet_transmit_length(length);

	ret &= write_reg(FIFO_ACCESS_REG, data, length);

	ret &= tx_mode_enable();

	uint8_t omfc = 0;

	// Loop until packet has been sent (device has left TX mode)
	do
	{
		if (timeout++ > 10000)
		{
			return false;
		}

		ret &= read_reg(OP_FUNC_CNTL_1, &omfc);
	} while ( (omfc & OP_CNTL1_MODE_TX_ON) == OP_CNTL1_MODE_TX_ON);

	return ret;
}

bool Rfm22b::clear_tx_fifo(void)
{
	bool ret = true;

	uint8_t op_func_cntl_2 = 0;

	// Read Operating and Function Control register
	ret &= read_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	// Clear ffclrtx bit
	op_func_cntl_2 &= ~0x01;

	uint8_t clear_tx_fifo = 0x01;
	ret &= write_reg(OP_FUNC_CNTL_2, &clear_tx_fifo);
	clear_tx_fifo = 0x00;
	ret &= write_reg(OP_FUNC_CNTL_2, &clear_tx_fifo);

	return ret;
}

bool Rfm22b::clear_rx_fifo(void)
{
	bool ret = true;

	uint8_t op_func_cntl_2 = 0;

	// Read Operating and Function Control register
	ret &= read_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	// Clear ffclrrx bit
	op_func_cntl_2 &= ~0x02;

	uint8_t clear_rx_fifo = 0x02;
	ret &= write_reg(OP_FUNC_CNTL_2, &clear_rx_fifo);
	clear_rx_fifo = 0x00;
	ret &= write_reg(OP_FUNC_CNTL_2, &clear_rx_fifo);

	return ret;
}

bool Rfm22b::set_packet_transmit_length(uint8_t length)
{
	return write_reg(TRANSMIT_PKT_LEN, &length);
}

bool Rfm22b::tx_mode_enable(bool enable)
{
	bool ret = true;

	uint8_t op_func_cntl_1 = 0;

	// Read Operating an Function Control 1 register
	ret &= read_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	if (enable)
	{
		// Set txon bit
		op_func_cntl_1 |= OP_CNTL1_MODE_TX_ON;
	}
	else
	{
		// Clear txon bit
		op_func_cntl_1 &= ~OP_CNTL1_MODE_TX_ON;
	}

	// Write new value to register
	ret &= write_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	return ret;
}

bool Rfm22b::rx_mode_enable(bool enable)
{
	bool ret = true;

	uint8_t op_func_cntl_1 = 0;

	// Read Operating an Function Control 1 register
	ret &= read_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	if (enable)
	{
		// Set rxon bit
		op_func_cntl_1 |= OP_CNTL1_MODE_RX_ON;
	}
	else
	{
		// Clear rxon bit
		op_func_cntl_1 &= ~OP_CNTL1_MODE_RX_ON;
	}

	// Write new value to register
	ret &= write_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	return ret;
}

bool Rfm22b::receive_bis(uint8_t *data, int* length)
{
	bool ret = true;
	uint32_t timeout = 0;

	ret &= clear_rx_fifo();

	ret &= rx_mode_enable();

	uint8_t omfc = 0;

	// Loop until packet has been sent (device has left RX mode)
	do
	{
		if (timeout++ > 10000)
		{
			// *length = 0;
			// ret = false;
			return false;
			// return true;
		}

		ret &= read_reg(OP_FUNC_CNTL_1, &omfc);
		// ret &= read_reg(INTERRUPT_STAT_1, &omfc);

	} while ((omfc & OP_CNTL1_MODE_RX_ON) == OP_CNTL1_MODE_RX_ON);
	// } while (!((omfc >> 1)& 1));

	uint8_t rx_len = 0;
	ret &= read_reg(RECEIVE_PKT_LEN, &rx_len);

	if (rx_len > 64)
	{
		rx_len = 64;
	}


	// ret &= read_reg(INTERRUPT_STAT_1, &omfc);
	// if ((omfc >> 1) & 1)
	// {
		ret &= read_reg(FIFO_ACCESS_REG, data, rx_len);
	// }

	*length = rx_len;

	return ret;
}

bool Rfm22b::get_rssi(uint8_t* rssi)
{
	return read_reg(RSSI_REG, rssi);
}
bool Rfm22b::set_preamble_detection(uint8_t n_nibble)
{
	bool ret = true;
	uint8_t preamble_det_cntl = 0;

	// Read Preamble Detection Control register
	ret &= read_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

	// Clearing preath bits
	preamble_det_cntl &= 0x07;

	preamble_det_cntl |= n_nibble << 3;

	// Write new value to register
	ret &= write_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

	return ret;
}

bool Rfm22b::set_header_length(uint8_t length)
{
	bool ret = true;

	uint8_t header_cntl2 = 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_2, &header_cntl2);

	// Clearing hdlen bits
	header_cntl2 &= ~0x30;

	header_cntl2 |= length << 4;

	// Write new register value
	ret &= write_reg(HEADER_CNTL_2, &header_cntl2);
	return ret;
}

bool Rfm22b::set_header_check(void)
{
	bool ret = true;

	uint8_t header_cntl1 = 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_1, &header_cntl1);

	// Clearing hdch bits
	header_cntl1 &= ~0x0F;

	header_cntl1 |= 0x0F;
	
	ret &= write_reg(HEADER_CNTL_1, &header_cntl1);
	return ret;
}

bool Rfm22b::set_check_header(uint8_t* chhd)
{
	return write_reg(CHECK_HEADER_3, chhd, 4);
}

bool Rfm22b::get_received_header(uint8_t* rx_header)
{
	bool ret = true;

	uint8_t rxhd[4] = {0};
	ret &= read_reg(RECEIVED_HEADER_3, rxhd, 4);

	*rx_header 		= rxhd[0];
	*(rx_header+1) 	= rxhd[1];
	*(rx_header+2) 	= rxhd[2];
	*(rx_header+3) 	= rxhd[3];

	return ret;
}

bool Rfm22b::set_syncword_length(uint8_t length)
{
	bool ret = true;

	if (length > 3)
	{
		length = 3;
	}

	uint8_t header_cntl2 = 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_2, &header_cntl2);

	// Clearing synclen bits
	header_cntl2 &= ~0x06;

	header_cntl2 |= length << 1;

	ret &= write_reg(Rfm22b::HEADER_CNTL_2, &header_cntl2);
	return ret;
}

bool Rfm22b::get_transmit_header(uint8_t* tx_header)
{
	bool ret = true;
	uint8_t txhd[4] = {0};

	ret &= read_reg(TRANSMIT_HEADER_3, txhd, 4);
	*tx_header 		= txhd[0];
	*(tx_header+1) 	= txhd[1];
	*(tx_header+2) 	= txhd[2];
	*(tx_header+3) 	= txhd[3];

	return ret;
}

bool Rfm22b::header_enable(void)
{
	bool ret = true;

	uint8_t hden[4] = {0xFF};
	ret &= write_reg(HEADER_EN_3, hden, 4);

	return ret;
}

bool Rfm22b::interrput_enable(uint8_t in1en, uint8_t in2en)
{
	bool ret = true;

	uint8_t inen[2] = {in1en, in2en};
	ret &= write_reg(INTERRUPT_EN_1, inen, 2);

	return ret;
}

bool Rfm22b::set_rssi_offset(uint8_t offset)
{
	bool ret = true;

	if (offset > 7)
	{
		offset = 7;
	}

	uint8_t preamble_det_cntl = 0;
	ret &= read_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

	// Clearing rssi_off bits
	preamble_det_cntl &= ~0x07;
	preamble_det_cntl |= offset & 0x07;

	ret &= write_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

	return ret;
}

bool Rfm22b::set_rssi_threshold(uint8_t threshold)
{
	return write_reg(RSSI_THRESH_CLR_CH, &threshold);
}

bool Rfm22b::get_battery_level(float* battery_level)
{
	bool ret = true;

	uint8_t batt_lvl = 0;

	ret &= read_reg(BATT_V_LEVEL, &batt_lvl);

	*battery_level = 1.7f + 50E-3 * (float)batt_lvl;

	return ret;
}

bool Rfm22b::set_lbd_threshold(uint8_t threshold)
{
	bool ret = true;

	uint8_t lbd_threshold = 0;

	// Read Low Battery Detection Threshold register
	ret &= read_reg(LBD_THRESHOLD, &lbd_threshold);

	// Clearing lbdt bits
	lbd_threshold &= ~0x1F;

	lbd_threshold |= threshold & 0x1F;

	// Write new value to register
	ret &= write_reg(LBD_THRESHOLD, &lbd_threshold);

	return ret;
}

bool Rfm22b::enable_low_battery_detection(bool enable)
{
	bool ret = true;
	uint8_t op_func_cntl_1 = 0;

	// Read Operating and Function Control 1 register
	ret &= read_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	if (enable)
	{
		// Set enlbd bit
		op_func_cntl_1 |= 0x40;
	}
	else
	{
		// Clear enlbd bit
		op_func_cntl_1 &= ~0x40;
	}

	// Write new value to register
	ret &= write_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	return ret;
}

bool Rfm22b::set_preamble_length(uint16_t length)
{
	bool ret = true;

	// Seperate length into two bytes
	uint8_t length_1 = (length & 0x100) >> 8;
	uint8_t length_0 = length & 0xFF;

	uint8_t header_cntl2 	= 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_2, &header_cntl2);

	header_cntl2 |= length_1;

	// Write new values to registers
	ret &= write_reg(HEADER_CNTL_2, &header_cntl2);
	ret &= write_reg(PREAMBLE_LEN_REG, &length_0);

	return ret;
}

bool Rfm22b::set_syncword(uint8_t* syncword)
{
	return write_reg(SYNC_WORD_3, syncword, 4);
}

int Rfm22b::transmit(uint8_t* tx_buffer, uint8_t tx_len)
{
	bool ret = true;
	uint32_t timeout = 0;

	const char* sep  = " ";
    const char* sep2 = "\t";
    const char* newline = "\r\n";
    uint64_t delay = 5;//25;

	clear_tx_fifo(); // Might have to be done outside
	// clear_rx_fifo(); // Merge those two?

    uint8_t device_status = 0;
	uint8_t interrupt_1 = 0;
	uint8_t interrupt_2 = 0;

	// Caping length
	if (tx_len > 64)
	{
		tx_len = 64;
	}

	ret &= set_packet_transmit_length(tx_len);

	// // Reading Interrupt
	ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
	ret &= read_reg(INTERRUPT_STAT_2, &interrupt_2);

	// Put message into FIFO
	ret &= write_reg(FIFO_ACCESS_REG, tx_buffer, tx_len);

	// // Check if TX overflow
	ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
	if ((interrupt_1 >> 7) & 1)
	{
		return -1;
	}

	tx_mode_enable();

	do
	{
		if (timeout++ > 10000)
		{
			tx_mode_enable(false);
			return -2;
		}
		ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
	} while (!((interrupt_1 >> 2) & 1));

	// while (1)
 //    {
 //        ret &= read_reg(DEVICE_STATUS, &device_status);   
 //        ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
 //        ret &= read_reg(INTERRUPT_STAT_2, &interrupt_2);

 //        for (int i = 0; i < 8; i++)
 //        {
 //            console.write((device_status >> (7-i)) & 1);
 //            time_keeper_delay_ms(delay);
 //            serial_.write((const uint8_t*)sep, sizeof(sep));
 //            time_keeper_delay_ms(delay);
 //        }

 //        serial_.write((const uint8_t*)sep2, sizeof(sep));
 //        time_keeper_delay_ms(delay);

 //        for (int i = 0; i < 8; i++)
 //        {
 //            console.write((interrupt_1 >> (7-i)) & 1);
 //            time_keeper_delay_ms(delay);
 //            serial_.write((const uint8_t*)sep, sizeof(sep));
 //            time_keeper_delay_ms(delay);
 //        }

 //        serial_.write((const uint8_t*)sep2, sizeof(sep));
 //        time_keeper_delay_ms(delay);

 //        for (int i = 0; i < 8; i++)
 //        {
 //            console.write((interrupt_2 >> (7-i)) & 1);
 //            time_keeper_delay_ms(delay);
 //            serial_.write((const uint8_t*)sep, sizeof(sep));
 //            time_keeper_delay_ms(delay);
 //        }

 //        console.write('S');
 //        time_keeper_delay_ms(delay);
 //        serial_.write((const uint8_t*)sep, sizeof(sep));
 //        time_keeper_delay_ms(delay);
 //        serial_.write((const uint8_t*)sep, sizeof(sep));
 //        time_keeper_delay_ms(delay);

 //        serial_.write((const uint8_t*)newline, sizeof(newline));
 //        time_keeper_delay_ms(delay);

 //        if ((interrupt_1 >> 2)&1)
 //        {
 //            break;
 //        }

 //        time_keeper_delay_ms(1);
 //    }

	if (ret)
	{
		return 1;
	}
	else
	{
		return -6;
	}
	// return ret;
}

int Rfm22b::receive(uint8_t* rx_buffer, uint8_t* rx_len)
{
	bool ret = true;
	uint32_t timeout = 0;

	const char* sep  = " ";
    const char* sep2 = "\t";
    const char* newline = "\r\n";
    uint64_t delay = 5;//25;

	clear_rx_fifo(); // Might have to be done outside
	// clear_tx_fifo();

    uint8_t device_status = 0;
	uint8_t interrupt_1 = 0;
	uint8_t interrupt_2 = 0;

	// // Reading Interrupt
	ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
	ret &= read_reg(INTERRUPT_STAT_2, &interrupt_2);

	rx_mode_enable();

	do
	{
		if (timeout++ > 10000)
		{
			rx_mode_enable(false);
			return -3;
		}
		ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
		// ret &= read_reg(OP_FUNC_CNTL_1, &interrupt_1);
        // time_keeper_delay_ms(1000);
	} while (!((interrupt_1 >> 1) & 1));
	// } while ((interrupt_1 & OP_CNTL1_MODE_RX_ON) == OP_CNTL1_MODE_RX_ON);

	// Get received packet length
	ret &= read_reg(RECEIVE_PKT_LEN, rx_len);

	// Caping length (multi packet not supported yet)
	if (*rx_len > 64)
	{
		*rx_len = 64;
	}

	// Copying message from FIFO to rx buffer
	ret &= read_reg(FIFO_ACCESS_REG, rx_buffer, *rx_len);

	// Check if RX underflow
	ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
	if ((interrupt_1 >> 7) & 1)
	{
		return -4;
	}

   //  while (1)
   //  {
   //      ret &= read_reg(DEVICE_STATUS, &device_status); 
   //      ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
   //      ret &= read_reg(INTERRUPT_STAT_2, &interrupt_2);

   //      for (int i = 0; i < 8; i++)
   //      {
   //          console.write((device_status >> (7-i)) & 1);
   //          time_keeper_delay_ms(delay);
   //          serial_.write((const uint8_t*)sep, sizeof(sep));
   //          time_keeper_delay_ms(delay);
   //      }

   //      serial_.write((const uint8_t*)sep2, sizeof(sep));
   //      time_keeper_delay_ms(delay);

   //      for (int i = 0; i < 8; i++)
   //      {
   //          console.write((interrupt_1 >> (7-i)) & 1);
   //          time_keeper_delay_ms(delay);
   //          serial_.write((const uint8_t*)sep, sizeof(sep));
   //          time_keeper_delay_ms(delay);
   //      }

   //      serial_.write((const uint8_t*)sep2, sizeof(sep));
   //      time_keeper_delay_ms(delay);

   //      for (int i = 0; i < 8; i++)
   //      {
   //          console.write((interrupt_2 >> (7-i)) & 1);
   //          time_keeper_delay_ms(delay);
   //          serial_.write((const uint8_t*)sep, sizeof(sep));
   //          time_keeper_delay_ms(delay);
   //      }

   //      console.write('R');
   //      time_keeper_delay_ms(delay);
   //      serial_.write((const uint8_t*)sep2, sizeof(sep));
   //      time_keeper_delay_ms(delay);
   //      serial_.write((const uint8_t*)sep, sizeof(sep));
   //      time_keeper_delay_ms(delay);

   //      serial_.write((const uint8_t*)newline, sizeof(newline));
   //      time_keeper_delay_ms(delay);

   //      if ((interrupt_1 >> 1)&1)
   //      {
   //      	// Get received packet length
			// ret &= read_reg(RECEIVE_PKT_LEN, rx_len);
   //          ret &= read_reg(FIFO_ACCESS_REG, rx_buffer, *rx_len);
   //          break;
   //      }

   //      time_keeper_delay_ms(1);
   //  }

	if (ret)
	{
		return 1;
	}
	else
	{
		return -7;
	}

	// return ret;
}