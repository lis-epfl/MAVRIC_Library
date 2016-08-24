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

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Rfm22b::Rfm22b(Spi& spi, Gpio& nss_gpio)://, const conf_t config):
spi_(spi),
nss_(nss_gpio)
{}

bool Rfm22b::init()
{
	bool success = true;

	// Initializing Slave Select GPIO
    success &= nss_.init();
    unselect_slave();

    // Reset rfm22b
    success &= reset();

    // Disable interrupts
    uint8_t dis_int = 0x00;
    success &= write_reg(INTERRUPT_EN_1, &dis_int);
    success &= write_reg(INTERRUPT_EN_2, &dis_int);

    // Test to see if device is responding
	uint8_t device_type_answer = 0x01;
	success &= read_reg(DEVICE_TYPE_REG, &device_type_answer);
	success &= device_type_answer == DEVICE_TYPE ? true : false;

	if (!success)
	{
		return success;
	}

	// Set Cristal Oscillator to get desired frequency
	uint8_t osc_load_cap = 0x7F;
	success &= write_reg(OSC_LOAD_CAP_REG, &osc_load_cap);

	// Disable low duty cycle mode
	uint8_t duty_cycle_mode = 0x00;
	success &= write_reg(OP_FUNC_CNTL_2, &duty_cycle_mode);

	// Set output clock to 1MHz
	uint8_t output_clock = CPU_OUTPUT_CLK_1MHZ;
	success &= write_reg(CPU_OUTPUT_CLK, &output_clock);

	// Set Ready Mode
	uint8_t set_mode = OP_CNTL1_MODE_IDLE_READY;
	success &= write_reg(OP_FUNC_CNTL_1, &set_mode);

	// Configure I/O ports
	uint8_t io_port_config 	= 0x00; // Default config
	uint8_t gpio0_config 	= 0xC0 | GPIOX_CONFIG_TX_STATE;
	uint8_t gpio1_config 	= 0xC0 | GPIOX_CONFIG_RX_STATE;
	uint8_t gpio2_config	= 0xC0 | GPIOX_CLR_CH_ASSESS;
	success &= write_reg(IO_PORT_CONFIG, &io_port_config);
	success &= write_reg(GPIO0_CONFIG_REG, &gpio0_config);
	success &= write_reg(GPIO1_CONFIG_REG, &gpio1_config);
	success &= write_reg(GPIO2_CONFIG_REG, &gpio2_config);

	time_keeper_delay_ms(1);

	// Set Data Source and Modulation Mode
	uint8_t fd_msb = 0;
	success &= read_reg(MOD_MODE_CNTL_2, &fd_msb);
	fd_msb  &= MOD_CNTL2_FD_MSB;
	uint8_t data_source = MOD_CNTL2_FIFO | MOD_CNTL2_GFSK | fd_msb;
	success &= write_reg(MOD_MODE_CNTL_2, &data_source);

	time_keeper_delay_ms(1);

	// Enabling TX/RX packet handling
	uint8_t data_access = ACCESS_TX_PACK_EN | ACCESS_RX_PACK_EN;
	success &= write_reg(DATA_ACCESS_CNTL, &data_access);


	// Set Preamble Length
	uint8_t preamble_length = 0x0C;
	success &= write_reg(PREAMBLE_LEN_REG, &preamble_length);

	// Set Preamble Detection Length
	uint8_t preamble_detection = 6 << 3;
	success &= write_reg(PREAMBLE_DET_CNTL, &preamble_detection);

	// Set header
	uint8_t header_control = 0xFF;
	success &= write_reg(HEADER_CNTL_1, &header_control);

	// Header enable
	uint8_t header_enable = 0xFF;
	success &= write_reg(0x46, &header_enable);
	success &= write_reg(0x45, &header_enable);
	success &= write_reg(0x44, &header_enable);
	success &= write_reg(0x43, &header_enable);

	// Header Length Syncword Length
	uint8_t hdlen_synclen = 0x40 | 0x06;
	success &= write_reg(HEADER_CNTL_2, &hdlen_synclen);

	time_keeper_delay_ms(1);

	// Set Syncword
	uint8_t sync_byte_1 = 0x2D;
	uint8_t sync_byte_2 = 0xD4;
	uint8_t sync_byte_3 = 0x4B;
	uint8_t sync_byte_4 = 0x59;
	success &= write_reg(SYNC_WORD_3, &sync_byte_1);
	success &= write_reg(SYNC_WORD_2, &sync_byte_2);
	success &= write_reg(SYNC_WORD_1, &sync_byte_3);
	success &= write_reg(SYNC_WORD_0, &sync_byte_4);

	// Set FIFO threshold
	uint8_t tx_fifo_thresh_h = 62;
	uint8_t tx_fifo_thresh_l = 32;
	uint8_t rx_fifo_thresh_h = 32;
	success &= write_reg(TX_FIFO_CNTL_1, &tx_fifo_thresh_h);
	success &= write_reg(TX_FIFO_CNTL_2, &tx_fifo_thresh_l);
	success &= write_reg(RX_FIFO_CNTL  , &rx_fifo_thresh_h);

	// Set Cristal Oscillator to get desired frequency
	osc_load_cap = 0x7F;
	success &= write_reg(OSC_LOAD_CAP_REG, &osc_load_cap);

	time_keeper_delay_ms(1);

	// ************ Set Nominal Carrier Frequency **************
	//

	// holds the hbsel (1 or 2)
	uint8_t hbsel;
	uint32_t frequency_hz = 430000000; //Hz

	if (frequency_hz < 480000000) {
		hbsel = 0;
	} else {
		hbsel = 1;
	}
	float freq_mhz = (float)(frequency_hz) / 1000000.0f;
	float xtal_freq_khz = 30000.0f;
	float sfreq = freq_mhz / (10.0f * (xtal_freq_khz / 30000.0f) * (1 + hbsel));
	uint32_t fb = (uint32_t) sfreq - 24 + (64 + 32 * hbsel);
	uint32_t fc = (uint32_t) ((sfreq - (uint32_t) sfreq) * 64000.0f);
	uint8_t fch = (fc >> 8) & 0xff;
	uint8_t fcl = fc & 0xff;

	// Set the frequency hopping step size, the step size is
	// 10MHz / 250 channels = 40khz, and the step size is
	// specified in 10khz increments.
	uint8_t freq_hop_step_size = 4;
	success &= write_reg(FREQ_HOP_STEP_SIZE, &freq_hop_step_size);

	// Frequency Hopping Channel
	uint8_t freq_hop_ch_sel = 0x00;
	success &= write_reg(FREQ_HOP_CH_SEL, &freq_hop_ch_sel);

	// No frequency Offset
	uint8_t frequency_offset = 0x00;
	success &= write_reg(FREQ_OFFSET_1, &frequency_offset);
	success &= write_reg(FREQ_OFFSET_2, &frequency_offset);

	// Set the carrier frequency
	uint8_t freq_band_sel = fb & 0xFF;
	uint8_t nrml_carr_freq_1 = fch;
	uint8_t nrml_carr_freq_0 = fcl;
	success &= write_reg(FREQ_BAND_SEL, &freq_band_sel);
	success &= write_reg(NRML_CARR_FREQ_1, &nrml_carr_freq_1);
	success &= write_reg(NRML_CARR_FREQ_0, &nrml_carr_freq_0);

	// ************ Set Data Rate **************
	//
	// if filter bandwidth
	uint8_t if_filter_bw = 0x01;
	success &= write_reg(0x1C, &if_filter_bw);

	// afc loop gearshift override
	uint8_t afc_loop_g_o = 0x40;
	success &= write_reg(0x1D, &afc_loop_g_o);

	// afc timing control
	uint8_t afc_timing_control = 0x0A;
	success &= write_reg(0x1E, &afc_timing_control);

	// Clock Recovery Gearshift Override
	uint8_t clk_rec_g_o = 0x03;
	success &= write_reg(0x1F, &clk_rec_g_o);

	// Clock Recovery Oversampling Ratio
	uint8_t clk_rec_ovrsamp_r = 0xA1;
	success &= write_reg(0x20, &clk_rec_ovrsamp_r);

	// Clock Recorvery Offset 2
	uint8_t clk_rec_offset_2 = 0x20;
	success &= write_reg(0x21, &clk_rec_offset_2);

	// Clock Recorvery Offset 1
	uint8_t clk_rec_offset_1 = 0x4E;
	success &= write_reg(0x22, &clk_rec_offset_1);

	// Clock Recorvery Offset 0
	uint8_t clk_rec_offset_0 = 0xA5;
	success &= write_reg(0x23, &clk_rec_offset_0);

	// Clock Recovery Timing Loop Gain 1
	uint8_t clk_rec_t_loop_gain_1 = 0x00;
	success &= write_reg(0x24, &clk_rec_t_loop_gain_1);

	// Clock Recovery Timing Loop Gain 0
	uint8_t clk_rec_t_loop_gain_0 = 0x34;
	success &= write_reg(0x25, &clk_rec_t_loop_gain_0);

	// agc override 1
	uint8_t agc_override_1 = 0x60;
	success &= write_reg(0x69, &agc_override_1);

	// afc limiter
	uint8_t afc_limiter = 0x1E;
	success &= write_reg(0x2A, &afc_limiter);

	// tx data rate 1
	uint8_t tx_data_rate_1 = 0x4E;
	success &= write_reg(0x6E, &tx_data_rate_1);

	// tx data rate 0
	uint8_t tx_data_rate_0 = 0xA5;
	success &= write_reg(0x6F, &tx_data_rate_0);

	// modulation mode control 1
	uint8_t mode_1 = 0x2C | MOD_CNTL1_WHITE_EN;
	success &= write_reg(MOD_MODE_CNTL_1, &mode_1);

	// modulation mode control 2
	uint8_t mode_2 = 0x23;
	success &= write_reg(MOD_MODE_CNTL_2, &mode_2);

	// frequency deviation
	uint8_t frequency_deviation = 0x30;
	success &= write_reg(FREQ_DEVIATION, &frequency_deviation);

	// ook counter val 1
	uint8_t ook_counter_val_1 = 0x00;
	success &= write_reg(OOK_COUNTER_VAL_1, &ook_counter_val_1);

	// ook counter val 2
	uint8_t ook_counter_val_2 = 0x00;
	success &= write_reg(OOK_COUNTER_VAL_2, &ook_counter_val_2);

	// Set Mode
	set_mode = OP_CNTL1_MODE_TX_ON;
	success &= write_reg(OP_FUNC_CNTL_1, &set_mode);

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

	return ret;
}

bool Rfm22b::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = true;

	// Disable interrupts
    uint8_t dis_int = 0x00;
    success &= write_reg(INTERRUPT_EN_1, &dis_int);
    success &= write_reg(INTERRUPT_EN_2, &dis_int);

    // Set TX power
    uint8_t tx_power = TX_POWER_LNA_SW | 0x00;
    success &= write_reg(TX_POWER, &tx_power);

    // Set TUNE mode
    uint8_t set_mode = OP_CNTL1_MODE_PPL_ON;
    success &= write_reg(MOD_MODE_CNTL_1, &set_mode);

	return ret;
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

    uint8_t reset_command = OP_CNTL1_SWRESET;

    // Write reset
    ret &= write_reg(OP_FUNC_CNTL_1, &reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}