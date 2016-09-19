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

// USED FOR INTERRUPTS BUT ONLY VALID FOR STM32 !!!
#include "hal/stm32/gpio_stm32.hpp"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>


static Rfm22b* handlers_ = 0;

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Rfm22b::Rfm22b(Spi& spi, Gpio& nss_gpio, Gpio& nirq_gpio, Led_gpio& led_irq)://, const conf_t config):
nirq_(nirq_gpio),
counter_var(0),
rx_success_(false),
spi_(spi),
nss_(nss_gpio),
// nirq_(nirq_gpio), // this is while nirq is public
rx_len_(0),
// rx_success_(false), // this is while rx_sucess_ is public
led_irq_(led_irq),
device_status_({0,0,0,0}),
coordinator_(false),
coordinator_id_(0),
device_id_(0),
ppm_send_mode_(false),
ppm_recv_mode_(false),
ppm_only_mode_(false),
one_way_link_(false),
datarate_(0),
packet_time_(0),
channels_({0}),
max_packet_len_(0),
destination_id_(0),
frequency_step_size_(0),
afc_correction_hz_(0),
rssi_dBm_(0),
trans_state_(TRANS_STATE_INITIALIZING),
irq_callback(0)
{}

bool Rfm22b::init()
{
	bool success = true;

	// Initialize Slave Select GPIO
    success &= nss_.init();
    unselect_slave();

    // Initialize nIRQ GPIO
    success &= nirq_.init();

    success &= config_device(0, 0, 250, 0, false, false, false); // Could increase datarate

    device_id_ = 0xE7E7E7E7;
    if (device_id_ == 0)
    {
    	device_id_ = 1;
    }

    success &= exti_init();

    //-->ecc library init missing


    // TODO: make an enum for datarate ? Because 0,1,2,3,... is not really explicit
    success &= config_device(3, 0, 250, 0, false, false, false); // not really need - try to remove this

    success &= set_transmission_power(1); // Warning: Taulabs do not set the power...

    success &= init_2(); // Sorry, I was not very inspired with names so, TODO: change name of function

    // success &= set_transmit_header(); // UNCOMMENT FOR ALL TESTS

    success &= set_rx_mode(); // COMMENT FOR ALL TESTS

    // prepare_receive(); // UNCOMMENT FOR LAST TEST ONLY

	return success;
}

bool Rfm22b::config_device(	int datarate,
							uint8_t min_chan,
							uint8_t max_chan,
							uint32_t coordinator_id,
							bool oneway,
							bool ppm_mode,
							bool ppm_only)
{
	bool ret = true;

	coordinator_ = coordinator_id == 0;
	ppm_mode = ppm_mode || ppm_only;

	coordinator_id_ = coordinator_id;

	ppm_send_mode_ = ppm_mode && coordinator_;
	ppm_recv_mode_ = ppm_mode && !coordinator_;

	// If datarate is so slow that we can only do PPM, force this
	if (ppm_mode && (datarate <= 0))
	{
		ppm_only = true;
	}

	ppm_only_mode_ = ppm_only;

	if (ppm_only)
	{
		one_way_link_ 	= true;
		datarate_ 		= 0;
	}
	else
	{
		one_way_link_ 	= false;
		datarate_ 		= datarate;
	}

	packet_time_ = (ppm_mode ? packet_time_ppm[datarate] : packet_time[datarate]);
	if (!one_way_link_)
	{
		packet_time_ *= 2;  // double the time to allow a send and receive in each slice
	}


	// Find the first N channels that meet the min/max criteria out of the random channel list.
	uint32_t crc = 0;
	const uint8_t CRC_INC = 0x39;
	if (coordinator_) {
		crc = update_byte(device_id_, CRC_INC);
	} else {
		crc = update_byte(coordinator_id_, CRC_INC);
	}

	uint8_t num_found = 0;
	while (num_found < num_channels[datarate])
	{
		// crc = PIOS_CRC_updateByte(crc, CRC_INC);
		uint8_t chan = min_chan + (crc % (max_chan - min_chan));

		if (chan < 250)
		{
			// skip any duplicates
			for (int32_t i = 0; i < num_found; i++)
			{
				if (channels_[i] == chan)
				{
					continue;
				}
			}
			channels_[num_found++] = chan;
		}
	}

	// Calculate the maximum packet length from the datarate.
	float bytes_per_period =
	    (float)data_rate[datarate_] * (float)(packet_time_ - 2) / 9000; // not 9000.0f ?

	max_packet_len_ = bytes_per_period - 6 - 4 -
	    4 - 1; // TX preamble nibbles / 2, syncword length, header bytes, length byte
	if (max_packet_len_ > 64) {
		max_packet_len_ = 64;
	}

	return true;
}

bool Rfm22b::init_2()
{
	bool success = true;

    // Reseting device
    success &= reset();

    // Read Status - Clear Interrupts
    read_status();

    // Disable all interrupts
    success &= clear_bit_in_reg(INTERRUPT_EN_1, CLEAR_ALL);
    success &= clear_bit_in_reg(INTERRUPT_EN_2, CLEAR_ALL);

    // Reading Device Type
	uint8_t device_type_answer = 0x00;
	success &= read_reg(DEVICE_TYPE_REG, &device_type_answer);
	success &= device_type_answer == DEVICE_TYPE ? true : false;

	// Reading Device Version
	uint8_t device_version_answer = 0x00;
	success &= read_reg(DEVICE_VERSION_REG, &device_version_answer);
	success &= device_version_answer == DEVICE_VERSION ? true : false;

	time_keeper_delay_ms(1);

	// Calibrate cristal oscillator to be exactly on frequency (different for every module ?)
	success &= set_cristal_osci_load_cap();

	// Disable low duty cycle mode
	clear_bit_in_reg(OP_FUNC_CNTL_2, OP_CNTL2_LOW_DC_MSK);

	// Set output clock to 1MHz
	uint8_t output_clock = CPU_OUTPUT_CLK_1MHZ;
	success &= write_reg(CPU_OUTPUT_CLK, &output_clock);

	// Set Ready Mode
	uint8_t set_mode = OP_CNTL1_MODE_IDLE_READY;
	success &= write_reg(OP_FUNC_CNTL_1, &set_mode);

	// Set I/O port configuration to default
	success &= clear_bit_in_reg(IO_PORT_CONFIG, CLEAR_ALL);

	// Configure GPIOS
    success &= set_gpio_function();

    time_keeper_delay_ms(1);

    // Set modulation Type
    success &= set_modulation_type();

    // Set modulation data source
    success &= set_modulation_data_source();

    // No TX data clock
    success &= set_data_clock_configuration();

    // Set RSSI threshold to -90 dBm
    success &= set_rssi_threshold(-90);

    time_keeper_delay_ms(1);

    // Enable TX and RX packet handlers and disable CRC
    success &= set_bit_in_reg(DATA_ACCESS_CNTL, ACCESS_RX_PACK_EN_MSK | ACCESS_TX_PACK_EN_MSK);
    success &= clear_bit_in_reg(DATA_ACCESS_CNTL, ACCESS_CRC_EN_MSK);

    // Preamble Configuration
    success &= set_preamble_length(12);
    success &= set_preamble_detection(6);

    // Set Header Control 1 register
    success &= set_header_check();
	success &= set_header_broadcast_address_check();

	// Enable header if device is bound
	success &= header_enable();

	// Set Received Header Check
    success &= set_check_header();

    // Set header and syncword length
    success &= set_header_length(HDR_CNTL2_HD_3210);
    success &= set_syncword_length(HDR_CNTL2_SYNC_3210);

    // Include packet length in package
    success &= clear_bit_in_reg(HEADER_CNTL_2, HDR_CNTL2_FIXPKLEN_MSK);

    time_keeper_delay_ms(1);

    // Set syncword
    success &= set_syncword();

    // Set TX and RX FIFOs threshold
    success &= set_fifos_threshold();

    // Set the frequency calibration ??
	success &= set_cristal_osci_load_cap();

	time_keeper_delay_ms(1);

	// Set nominal carrier frequency and frequency hopping step size
	set_nominal_carrier_frequency(433000000, 0);

	// Set datarate
	set_datarate();

	return success;
}

bool Rfm22b::attach(serial_interrupt_callback_t func)
{
	irq_callback = func;
	return true;
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
	// This is temporary for tests to work
	return rx_len_;
}

uint32_t Rfm22b::writeable(void)
{
	return 0;
}

bool Rfm22b::read(uint8_t* bytes, const uint32_t size)
{
	bool ret = true;

	return true;
}

bool Rfm22b::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = true;

	// disable interruption while transmitting
	isr_enable(false);

	rx_mode_enable(false);
	// time_keeper_delay_us(200);
	ret &= transmit((uint8_t*)bytes, size);
	if (!ret)
	{
		isr_enable();
		// Transmission error
		return false;
	}

	// Going back to RX mode
	ret &= prepare_receive();

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
    for (uint8_t i = 0; i < 50; ++i) {
		// Read the status registers
		read_status();

		// Is the chip ready?
		if (device_status_.int_status_2.chip_ready) {
			break;
		}
		time_keeper_delay_ms(1);
	}

    return ret;
}

bool Rfm22b::temp_sens_config(void)
{
	bool ret = true;

	// ADC used to sample the temperature sensor
	uint8_t adc_config = ADC_SOURCE_SEL_TEMP_SENSOR;
	ret &= write_reg(ADC_CONFIG, &adc_config);

	// ADC offset
	uint8_t adc_offset = 0x00;
	ret &= write_reg(ADC_SENS_AMP_OFST, &adc_offset);

	// Temperature sensor calibration
	uint8_t temp_sens_calib = TEMP_SENS_CALIB_0;
	ret &= write_reg(TEMP_SENS_CNTL, &temp_sens_calib);

	// Temperature sensor offset
	uint8_t temp_offset = 0x00;
	ret &= write_reg(TEMP_VAL_OFST, &temp_offset);

	// Start an ADC conversion
	adc_config |= ADC_START;
	ret &= write_reg(ADC_CONFIG, &adc_config);

	return ret;
}


bool Rfm22b::set_nominal_carrier_frequency(uint32_t frequency_hz, uint8_t init_chan)
{
	bool ret = true;

	uint8_t freq_hop_step_size = 4;
	uint8_t hbsel;

	if (frequency_hz < 480000000)
	{
		hbsel = 0;
	}
	else
	{
		hbsel = 1;
	}

	float freq_mhz 		= (float)(frequency_hz) / 1000000.0f;
	float xtal_freq_khz = 30000.0f;
	float sfreq 		= freq_mhz / (10.0f * (xtal_freq_khz / 30000.0f) * (1 + hbsel));

	uint32_t fb = (uint32_t) sfreq - 24 + (64 + 32 * hbsel);
	uint32_t fc = (uint32_t) ((sfreq - (uint32_t) sfreq) * 64000.0f);
	uint8_t fch = (fc >> 8) & 0xff;
	uint8_t fcl = fc & 0xff;

	// Set frequency hopping step size
	ret &= write_reg(FREQ_HOP_STEP_SIZE, &freq_hop_step_size);

	frequency_step_size_ = 156.25f * hbsel;

	// Set frequency hopping channel number
	ret &= write_reg(FREQ_HOP_CH_SEL, &init_chan);

	// No frequency offset
	ret &= clear_bit_in_reg(FREQ_OFFSET_1, CLEAR_ALL);
	ret &= clear_bit_in_reg(FREQ_OFFSET_2, CLEAR_ALL);

	uint8_t fbs = 0;

	// Read frequency band select register
	ret &= read_reg(FREQ_BAND_SEL, &fbs);

	// Clearing fb, hbsel and sbsel bits
	fbs &= 0x80;

	fbs |= fb & 0xff; // Taulabs forget to add hbsel here !!!!
	// fbs |= hbsel << 5 | fb & 0xff; // Should be this, works without for low frequencies but is better to include it

	ret &= write_reg(FREQ_BAND_SEL, &fbs);
	ret &= write_reg(NRML_CARR_FREQ_1, &fch);
	ret &= write_reg(NRML_CARR_FREQ_0, &fcl);

	return ret;
}

bool Rfm22b::set_modulation_type(void)
{
	bool ret = true;

	uint8_t mmc2 = 0;

	// Read MMC2 reg
	ret &= read_reg(MOD_MODE_CNTL_2, &mmc2);

	// Clear the modtyp bits
	mmc2 &= ~MOD_CNTL2_MODTYP_MSK;

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
	mmc2 &= ~MOD_CNTL2_DTMOD_MSK;

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
	mmc2 &= ~MOD_CNTL2_TRCLK_MSK;
	
	// Set the desired data source
	mmc2 |= 0x00 << 6; 				// here 0 but we could let user choose?

	// Write new value to register
	ret &= write_reg(MOD_MODE_CNTL_2, &mmc2);

	return ret;
}

bool Rfm22b::set_transmission_power(uint8_t power_dBm)
{
	bool ret = true;

	uint8_t tx_power_reg = 0;
	
	// Read TX power register
	ret &= read_reg(TX_POWER, &tx_power_reg);

	// Clear txpow bits
	tx_power_reg &= ~TX_POW_TXPOW_MSK;

	// Convert and set the desired tx power
	tx_power_reg |= (power_dBm+1) / 3 & TX_POW_TXPOW_MSK;

	// Write new value to register
	ret &= write_reg(TX_POWER, &tx_power_reg);

	return ret;
}

bool Rfm22b::set_gpio_function(void)
{
	bool ret = true;

	uint8_t gpio0 = 0;
	uint8_t gpio1 = 0;
	uint8_t gpio2 = 0;

	// Read GPIOx config registers
	ret &= read_reg(GPIO0_CONFIG_REG, &gpio0);
	ret &= read_reg(GPIO1_CONFIG_REG, &gpio1);
	ret &= read_reg(GPIO2_CONFIG_REG, &gpio2);

	// Clear GPIOx bits
	gpio0 &= ~(GPIOX_PIN_FUNC_SEL_MSK | GPIOX_DRVG_CAPA_MSK);
	gpio1 &= ~(GPIOX_PIN_FUNC_SEL_MSK | GPIOX_DRVG_CAPA_MSK);
	gpio2 &= ~(GPIOX_PIN_FUNC_SEL_MSK | GPIOX_DRVG_CAPA_MSK);

	// Set the GPIOx bits
	gpio0 |= GPIOX_CONFIG_TX_STATE | GPIOX_DRVG_LVL_3;
	gpio1 |= GPIOX_CONFIG_RX_STATE | GPIOX_DRVG_LVL_3;
	gpio2 |= GPIOX_CLR_CH_ASSESS   | GPIOX_DRVG_LVL_3;

	// Write new value to register
	ret &= write_reg(GPIO0_CONFIG_REG, &gpio0);
	ret &= write_reg(GPIO1_CONFIG_REG, &gpio1);
	ret &= write_reg(GPIO2_CONFIG_REG, &gpio2);

	return ret;
}

bool Rfm22b::set_transmit_header(void)
{
	uint32_t id = get_destination_id();
	uint8_t txhd[4] = 	{ 	id 		& 0xFF,
						(id >> 8)  	& 0xFF,
						(id >> 16) 	& 0xFF,
						(id >> 24) 	& 0xFF};

	return write_reg(TRANSMIT_HEADER_3, txhd, 4);
}

bool Rfm22b::clear_tx_fifo(void)
{
	bool ret = true;

	uint8_t op_func_cntl_2 = 0;

	// Read Operating and Function Control register
	ret &= read_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	// Set ffclrtx bit
	op_func_cntl_2 |= OP_CNTL2_FFCLRTX_MSK;
	ret &= write_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	// Clear ffclrtx bit
	op_func_cntl_2 &= ~OP_CNTL2_FFCLRTX_MSK;
	ret &= write_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	return ret;
}

bool Rfm22b::clear_rx_fifo(void)
{
	bool ret = true;

	uint8_t op_func_cntl_2 = 0;

	// Read Operating and Function Control register
	ret &= read_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	// Set ffclrrx bit
	op_func_cntl_2 |= OP_CNTL2_FFCLRRX_MSK;
	ret &= write_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	// Clear ffclrrx bit
	op_func_cntl_2 &= ~OP_CNTL2_FFCLRRX_MSK;
	ret &= write_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

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
		op_func_cntl_1 |= OP_CNTL1_MODE_TX_EN_MSK;
	}
	else
	{
		// Clear txon bit
		op_func_cntl_1 &= ~OP_CNTL1_MODE_TX_EN_MSK;
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
		op_func_cntl_1 |= OP_CNTL1_MODE_RX_EN_MSK;
	}
	else
	{
		// Clear rxon bit
		op_func_cntl_1 &= ~OP_CNTL1_MODE_RX_EN_MSK;
	}

	// Write new value to register
	ret &= write_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

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

	// Clear preath bits
	preamble_det_cntl &= ~PREAM_DET_CNTL_PREATH_MSK;

	// Set preath bits
	preamble_det_cntl |= (n_nibble << 3) & PREAM_DET_CNTL_PREATH_MSK;

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
	header_cntl2 &= ~HDR_CNTL2_HDLEN_MSK;

	header_cntl2 |= (length << 4) & HDR_CNTL2_HDLEN_MSK;

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
	header_cntl1 &= ~HDR_CNTL1_HDCH_MSK;

	header_cntl1 |= 0x0F & HDR_CNTL1_HDCH_MSK; // TODO: make the value choosable by user if useful
	
	ret &= write_reg(HEADER_CNTL_1, &header_cntl1);
	return ret;
}

bool Rfm22b::set_header_broadcast_address_check(void)
{
	bool ret = true;

	uint8_t header_cntl1 = 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_1, &header_cntl1);

	// Clearing hdch bits
	header_cntl1 &= ~HDR_CNTL1_BCEN_MSK;

	header_cntl1 |= 0xF0; // TODO: make the value choosable by user if useful
	
	ret &= write_reg(HEADER_CNTL_1, &header_cntl1);
	return ret;
}

bool Rfm22b::set_check_header(void)
{
	uint32_t id = get_destination_id();
	uint8_t chhd[4] = 	{ 	id 		& 0xFF, // causes warning, but how to fix them?
						(id >> 8)  	& 0xFF,
						(id >> 16) 	& 0xFF,
						(id >> 24) 	& 0xFF};

	return write_reg(CHECK_HEADER_3, chhd, 4);
}

bool Rfm22b::get_received_header(uint8_t* rx_header)
{
	bool ret = true;

	uint8_t rxhd[4] = {0};
	ret &= read_reg(RECEIVED_HEADER_3, rxhd, 4);

	*rx_header 		= rxhd[0]; // TODO: find a better way to do this...
	*(rx_header+1) 	= rxhd[1];
	*(rx_header+2) 	= rxhd[2];
	*(rx_header+3) 	= rxhd[3];

	return ret;
}

bool Rfm22b::set_syncword_length(uint8_t length)
{
	bool ret = true;

	uint8_t header_cntl2 = 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_2, &header_cntl2);

	// Clear synclen bits
	header_cntl2 &= ~HDR_CNTL2_SYNCLEN_MSK;

	// Set synclen bits
	header_cntl2 |= (length << 1) & HDR_CNTL2_SYNCLEN_MSK;

	ret &= write_reg(Rfm22b::HEADER_CNTL_2, &header_cntl2);

	return ret;
}

bool Rfm22b::get_transmit_header(uint8_t* tx_header)
{
	bool ret = true;
	uint8_t txhd[4] = {0};

	ret &= read_reg(TRANSMIT_HEADER_3, txhd, 4);

	*tx_header 		= txhd[0];	// TODO: find a better way to do this...
	*(tx_header+1) 	= txhd[1];
	*(tx_header+2) 	= txhd[2];
	*(tx_header+3) 	= txhd[3];

	return ret;
}

bool Rfm22b::header_enable(void)
{
	bool ret = true;

	uint8_t header_mask = (get_destination_id() == 0xFFFFFFFF ? 0 : 0xFF);

	uint8_t hden[4] = {header_mask};
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

	uint8_t preamble_det_cntl = 0;
	ret &= read_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

	// Clear rssi_off bits
	preamble_det_cntl &= ~PREAM_DET_CNTL_RSSI_OFF_MSK;

	// Set rssi_off bits
	preamble_det_cntl |= offset & PREAM_DET_CNTL_RSSI_OFF_MSK;

	ret &= write_reg(PREAMBLE_DET_CNTL, &preamble_det_cntl);

	return ret;
}

bool Rfm22b::set_rssi_threshold(uint8_t threshold_dBm)
{
	// Convert threshold
	uint8_t threshold = (threshold_dBm + 122)*2;

	return write_reg(RSSI_THRESH_CLR_CH, &threshold);
}

bool Rfm22b::get_battery_level(float* battery_level)
{
	bool ret = true;

	uint8_t batt_lvl = 0;

	ret &= read_reg(BATT_V_LEVEL, &batt_lvl);

	// Convert ADC value in Volt
	*battery_level = 1.7f + 50E-3 * (float)batt_lvl;

	return ret;
}

bool Rfm22b::set_lbd_threshold(float threshold_V)
{
	bool ret = true;

	uint8_t lbd_threshold = 0;

	// Unit conversion
	uint8_t threshold = (threshold_V - 1.7f) / 50E-3;

	// Read Low Battery Detection Threshold register
	ret &= read_reg(LBD_THRESHOLD, &lbd_threshold);

	// Clear lbdt bits
	lbd_threshold &= ~LBD_THRESH_LBDT_MSK;

	// Set lbdt bits
	lbd_threshold |= threshold & LBD_THRESH_LBDT_MSK;

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
		op_func_cntl_1 |= OP_CNTL1_MODE_LBD_EN_MSK;
	}
	else
	{
		// Clear enlbd bit
		op_func_cntl_1 &= ~OP_CNTL1_MODE_LBD_EN_MSK;
	}

	// Write new value to register
	ret &= write_reg(OP_FUNC_CNTL_1, &op_func_cntl_1);

	return ret;
}

bool Rfm22b::set_preamble_length(uint16_t n_nibbles)
{
	bool ret = true;

	// Seperate length into two bytes
	uint8_t length_1 = (n_nibbles & 0x100) >> 8;
	uint8_t length_0 = n_nibbles & 0xFF;

	uint8_t header_cntl2 	= 0;

	// Read Header Control 2 register
	ret &= read_reg(HEADER_CNTL_2, &header_cntl2);

	header_cntl2 |= length_1 & HDR_CNTL2_PREALEN_MSK;

	// Write new values to registers
	ret &= write_reg(HEADER_CNTL_2, &header_cntl2);
	ret &= write_reg(PREAMBLE_LEN_REG, &length_0);

	return ret;
}

bool Rfm22b::set_syncword(void)
{
	uint8_t syncword[4] = {0x2D, 0xD4, 0x4B, 0x59}; // Let the user choose? Or store value somewhere else?

	return write_reg(SYNC_WORD_3, syncword, 4);
}

bool Rfm22b::set_datarate(void)
{

	bool ret = true;

	bool data_whitening = true;

	// if filter bandwidth
	uint8_t if_filter_bw = reg_1C[datarate_];
	ret &= write_reg(IF_FILTER_BW, &if_filter_bw);

	// afc loop gearshift override
	uint8_t afc_loop_g_o = reg_1D[datarate_];
	ret &= write_reg(AFC_LOOP_GS_OVRRD, &afc_loop_g_o);

	// afc timing control
	uint8_t afc_timing_control = reg_1E[datarate_];
	ret &= write_reg(AFC_TIMING_CNTL, &afc_timing_control);

	// Clock Recovery Gearshift Override
	uint8_t clk_rec_g_o = reg_1F[datarate_];
	ret &= write_reg(CLK_REC_GS_OVRRD, &clk_rec_g_o);

	// Clock Recovery Oversampling Ratio
	uint8_t clk_rec_ovrsamp_r = reg_20[datarate_];
	ret &= write_reg(CLK_REC_OVRSMP_RT, &clk_rec_ovrsamp_r);

	// Clock Recorvery Offset 2
	uint8_t clk_rec_offset_2 = reg_21[datarate_];
	ret &= write_reg(CLK_REC_OFST_2, &clk_rec_offset_2);

	// Clock Recorvery Offset 1
	uint8_t clk_rec_offset_1 = reg_22[datarate_];
	ret &= write_reg(CLK_REC_OFST_1, &clk_rec_offset_1);

	// Clock Recorvery Offset 0
	uint8_t clk_rec_offset_0 = reg_23[datarate_];
	ret &= write_reg(CLK_REC_OFST_0, &clk_rec_offset_0);

	// Clock Recovery Timing Loop Gain 1
	uint8_t clk_rec_t_loop_gain_1 = reg_24[datarate_];
	ret &= write_reg(CLK_REC_TIM_LPG_1, &clk_rec_t_loop_gain_1);

	// Clock Recovery Timing Loop Gain 0
	uint8_t clk_rec_t_loop_gain_0 = reg_25[datarate_];
	ret &= write_reg(CLK_REC_TIM_LPG_0, &clk_rec_t_loop_gain_0);

	// agc override 1
	uint8_t agc_override_1 = reg_69[datarate_];
	ret &= write_reg(AGC_OVRRD_1, &agc_override_1);

	// afc limiter
	uint8_t afc_limiter = reg_2A[datarate_];
	ret &= write_reg(AFC_LIMITER, &afc_limiter);

	// tx data rate 1
	uint8_t tx_data_rate_1 = reg_6E[datarate_];
	ret &= write_reg(TX_DATA_RATE_1, &tx_data_rate_1);

	// tx data rate 0
	uint8_t tx_data_rate_0 = reg_6F[datarate_];
	ret &= write_reg(TX_DATA_RATE_0, &tx_data_rate_0);

	// modulation mode control 1
	uint8_t mode_1 = reg_70[datarate_];
	if (!data_whitening) {
		mode_1 &= ~MOD_CNTL1_WHITE_EN;
	} else {
		mode_1 |= MOD_CNTL1_WHITE_EN;
	}
	ret &= write_reg(MOD_MODE_CNTL_1, &mode_1);

	// modulation mode control 2
	uint8_t mode_2 = reg_71[datarate_];
	ret &= write_reg(MOD_MODE_CNTL_2, &mode_2);

	// frequency deviation
	uint8_t frequency_deviation = reg_72[datarate_];
	ret &= write_reg(FREQ_DEVIATION, &frequency_deviation);

	// cpuu ?? Warning: reserved bits !!!!!!
	uint8_t cpuu = reg_58[datarate_];
	ret &= write_reg(0x58, &cpuu);

	// ook counter val 1
	uint8_t ook_counter_val_1 = 0x00;
	ret &= write_reg(OOK_COUNTER_VAL_1, &ook_counter_val_1);

	// ook counter val 2
	uint8_t ook_counter_val_2 = 0x00;
	ret &= write_reg(OOK_COUNTER_VAL_2, &ook_counter_val_2);

	return ret;
}

int Rfm22b::transmit(uint8_t* tx_buffer, uint8_t tx_len)
{
	bool ret = true;
	// uint8_t interrupt_1 = 0;

	// ret &= interrput_enable(0x84,0x00);
 //    ret &= clear_tx_fifo();

 //    // Caping length to 64 bytes
 //    if (tx_len > 64)
 //    {
 //        tx_len = 64;
 //    }

 //   	ret &= set_packet_transmit_length(tx_len);

	// // Reading Interrupt
 //    ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);

 //    // Put message into FIFO
 //    ret &= write_reg(FIFO_ACCESS_REG, tx_buffer, tx_len);

 //    // Check if TX overflow
 //    ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
 //    if ((interrupt_1 >> 7) & 1)
 //    {
 //        return false;
 //    }

 //    tx_mode_enable();

 //    while (1)
 //        {
 //            if (!nirq_.read())
 //            {
 //                ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
 //                if ((interrupt_1 >> 2) & 1)
 //                {
 //                	// Packet sent
 //                    break;
 //                }
 //            }
 //        }

	return ret;
}

int Rfm22b::prepare_receive(void)
{
	bool ret = true;
	
	// Going back to RX mode
	ret &= interrput_enable(0x93,0x00);

    ret &= clear_rx_fifo();

    uint8_t interrupt_1 = 0;

    // Reading Interrupt
    ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);

    rx_mode_enable();

    // enable interruption while receiving
	isr_enable();

	return ret;
}

void Rfm22b::isr_enable(bool enable)
{
	if (enable)
	{
		exti_enable_request(EXTI2); // This is only valid for STM32 and for SPARKY board
	}								// TODO: make this independent of board and architecture
	else
	{
		exti_disable_request(EXTI2);
	}

	return;
}

bool Rfm22b::rx_mulit_packet_en(bool enable)
{
	bool ret = true;

	uint8_t op_func_cntl_2 = 0;

	// Read Operating and Function Control register
	ret &= read_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	if (enable)
	{
		// Set rxmpk bit
		op_func_cntl_2 |= OP_CNTL2_RXMPK_MSK;
	}
	else
	{
		// Clear rxmpk bit
		op_func_cntl_2 &= ~OP_CNTL2_RXMPK_MSK;
	}

	// Write new value to register
	ret &= write_reg(OP_FUNC_CNTL_2, &op_func_cntl_2);

	return ret;
}

bool Rfm22b::receive(void)
{
	bool ret = true;
	uint8_t interrupt_1 = 0;

	led_irq_.toggle();
    ret &= read_reg(RECEIVE_PKT_LEN, &rx_len_);

        // Caping length (multi packet not supported yet)
        if (rx_len_ > 64)
        {
            rx_len_ = 64;
        }

    // Copying message from FIFO to rx buffer
    ret &= read_reg(FIFO_ACCESS_REG, rx_buffer_, rx_len_);

    // Check if RX underflow
    ret &= read_reg(INTERRUPT_STAT_1, &interrupt_1);
    if ((interrupt_1 >> 7) & 1)
    {
        ret = false;
    }

    rx_success_ = ret;
    prepare_receive();
	return true;
}

bool Rfm22b::read_status(void)
{
	bool ret = true;

	uint8_t dev_stat 	= 0;
	uint8_t ezmac_stat 	= 0;
	uint8_t int_stat[2] = {0};

	// Read device status, Interrupt Status 1 & 2 and EzMAC Status
	ret &= read_reg(DEVICE_STATUS, &dev_stat);
	ret &= read_reg(INTERRUPT_STAT_1, int_stat, 2);
	ret &= read_reg(EZMAC_STATUS, &dev_stat);

	// Store status
	device_status_.device_status_reg.raw 	= dev_stat;
	device_status_.int_status_1.raw 		= int_stat[0];
	device_status_.int_status_2.raw 		= int_stat[1];
	device_status_.ezmac_status.raw			= ezmac_stat;

	if (device_status_.int_status_2.poweron_reset)
	{
		return false;
	}

	return ret;
}

bool Rfm22b::set_cristal_osci_load_cap(void)
{

	// Set Cristal Oscillator to get desired frequency
	uint8_t osc_load_cap = 0x7F; // 12.5pF // Let user choose??

	return write_reg(OSC_LOAD_CAP_REG, &osc_load_cap);
}

bool Rfm22b::set_bit_in_reg(uint8_t reg, uint8_t mask)
{
	bool ret = true;

	uint8_t register_val = 0;

	// Read register
	ret &= read_reg(reg, &register_val);

	// Set bit(s) in register
	register_val |= mask;

	// Write new value to register
	ret &= write_reg(reg, &register_val);

	return ret;
}

bool Rfm22b::clear_bit_in_reg(uint8_t reg, uint8_t mask)
{
	bool ret = true;

	uint8_t register_val = 0;

	// Read register
	ret &= read_reg(reg, &register_val);

	// Clear bit(s) in register
	register_val &= ~mask;

	// Write new value to register
	ret &= write_reg(reg, &register_val);
	
	return ret;
}

uint32_t Rfm22b::get_destination_id(void)
{
	if (coordinator_)
	{
		// Device is a coordinator
		return device_id_;
	}
	else if (coordinator_id_)
	{
		// Device is not a coordinator but is bound to one
		return coordinator_id_;
	}
	else
	{
		// Unbound device
		return 0xFFFFFFFF;
	}
}

bool Rfm22b::set_fifos_threshold(void)
{
	bool ret = true;

	uint8_t tx_fifo_cntl1 = 0;
	uint8_t tx_fifo_cntl2 = 0;
	uint8_t rx_fifo_cntl  = 0;

	// Read TX/RX FIFO control registers
	ret &= read_reg(TX_FIFO_CNTL_1	, &tx_fifo_cntl1);
	ret &= read_reg(TX_FIFO_CNTL_2	, &tx_fifo_cntl2);
	ret &= read_reg(RX_FIFO_CNTL 	, &rx_fifo_cntl);

	// Clear txfaethr, txafthr and rxafthr bits
	tx_fifo_cntl1 &= ~TX_FIFO_CNTL1_TXAFTHR_MSK;
	tx_fifo_cntl2 &= ~TX_FIFO_CNTL2_TXFAETHR_MSK;
	rx_fifo_cntl  &= ~RX_FIFO_CNTL_RXAFTHR_MSK;

	// Set txfaethr, txafthr and rxafthr bits with desired val
	tx_fifo_cntl1 |= 62; // TODO: Store this values somewhere else
	tx_fifo_cntl2 |= 32;
	rx_fifo_cntl  |= 32;

	// Write new values to registers
	ret &= write_reg(TX_FIFO_CNTL_1	, &tx_fifo_cntl1);
	ret &= write_reg(TX_FIFO_CNTL_2	, &tx_fifo_cntl2);
	ret &= write_reg(RX_FIFO_CNTL 	, &rx_fifo_cntl);

	return ret;
}

uint8_t Rfm22b::update_byte(uint8_t crc, const uint8_t data)
{
	return crc_table[crc ^ data];
}

bool Rfm22b::exti_init(void)
{
	bool ret = true;

	nvic_enable_irq(NVIC_EXTI2_IRQ);				// TODO: make this architecture and board independent
    exti_select_source(EXTI2, GPIO_STM32_PORT_D);
    exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);

    handlers_ = this;

    isr_enable(false);

	return ret;
}

bool Rfm22b::set_rx_mode(void)
{
	bool ret = true;

	// disable all interrupts
	clear_bit_in_reg(INTERRUPT_EN_1, CLEAR_ALL);
	clear_bit_in_reg(INTERRUPT_EN_2, CLEAR_ALL);

	// Switch to TUNE mode
	set_bit_in_reg(OP_FUNC_CNTL_1, OP_CNTL1_MODE_PPL_EN_MSK);

	rx_buffer_wr_ = 0;

	// tx_data_wr_ = 0; // Don't know what those are
	// tx_data_rd_ = 0;

	clear_rx_fifo();
	clear_tx_fifo();

	// Enable RX interrupts
	set_bit_in_reg(INTERRUPT_EN_1,	INT_EN1_ICRCERROR_MSK	|
									INT_EN1_IPKVALID_MSK 	|
									INT_EN1_IRXFFAFULL_MSK	|
									INT_EN1_IFFERR_MSK);

	set_bit_in_reg(INTERRUPT_EN_2, 	INT_EN2_IPREAVAL_MSK	|
									INT_EN2_ISWDET_MSK);

	// Enable receiver
	set_bit_in_reg(OP_FUNC_CNTL_1, 	OP_CNTL1_MODE_RX_EN_MSK 	|
									OP_CNTL1_MODE_PPL_EN_MSK);

	return ret;
}

bool Rfm22b::process_rx(void)
{
	bool ret = true;

	// Read the device status registers
	if (!read_status())
	{
		rx_failure();
		return false; // int failure
	}

	// FIFO under/over flow error.  Restart RX mode.
	if (device_status_.int_status_1.fifo_underoverflow_error ||
		device_status_.int_status_1.crc_error)
	{
		rx_failure();
		return false; // int failure
	}

	// Valid packet received
	if (device_status_.int_status_1.valid_packet_received)
	{

		// Read the total length of the packet data
		uint8_t len = 0;
		ret &= read_reg(RECEIVE_PKT_LEN, &len);

		// The received packet is going to be larger than the receive buffer
		if (len > max_packet_len_)
		{
			rx_failure();
			return false; // int failure
		}

		// There must still be data in the RX FIFO we need to get
		if (rx_buffer_wr_ < len)
		{
			int32_t bytes_to_read = len - rx_buffer_wr_;

			// Fetch the data from the RX FIFO
			rx_buffer_wr_ += read_reg(FIFO_ACCESS_REG, rx_buffer_, bytes_to_read) == 0 ? bytes_to_read : 0;

			// Taulabs does something more, they put the read data not at first element?
			//
			// PIOS_SPI_TransferByte(rfm22b_dev->spi_id, RFM22_fifo_access & 0x7F);
			// rx_buffer_wr_ +=
			    // (PIOS_SPI_TransferBlock(rfm22b_dev->spi_id, OUT_FF,
			    //   (uint8_t *) & rx_buffer[rfm22b_dev->rx_buffer_wr],
			    //   bytes_to_read, NULL) == 0) ? bytes_to_read : 0;
		}

		// Read the packet header (destination ID)
		uint8_t received_header[4] = {0};
		get_received_header(received_header);

		rx_destination_id_  =  received_header[3];
		rx_destination_id_ |= (received_header[2] << 8);
		rx_destination_id_ |= (received_header[1] << 16);
		rx_destination_id_ |= (received_header[0] << 24);

		// Is there a length error?
		if (rx_buffer_wr_ != len)
		{
			rx_failure();
			return false; // int failure
		}

		// Increment the total byte received count.
		// stats.rx_byte_count += rx_buffer_wr_;

		// We're finished with Rx mode
		trans_state_ = TRANS_STATE_TRANSITION;

	}
	else if (device_status_.int_status_1.rx_fifo_almost_full)
	{
		// RX FIFO almost full, it needs emptying
		// read data from the rf chips FIFO buffer

		// Read the total length of the packet data
		uint8_t len = 0;
		ret &= read_reg(RECEIVE_PKT_LEN, &len);

		// The received packet is going to be larger than the specified length
		if ((rx_buffer_wr_ + 32) > len)
		{
			rx_failure();
			return false; // int failure
		}

		// The received packet is going to be larger than the receive buffer
		if ((rx_buffer_wr_ + 32) > max_packet_len_)
		{
			rx_failure();
			return false; // int failure
		}

		// Fetch the data from the RX FIFO
		rx_buffer_wr_ += read_reg(FIFO_ACCESS_REG, rx_buffer_, 32) == 0 ? 32 : 0;

		// Taulabs does something more, they put the read data not at first element?
		//
		// PIOS_SPI_TransferByte(rfm22b_dev->spi_id, RFM22_fifo_access & 0x7F);
		// rfm22b_dev->rx_buffer_wr += (PIOS_SPI_TransferBlock(rfm22b_dev->spi_id, OUT_FF,
		//       (uint8_t *) & rx_buffer[rfm22b_dev->rx_buffer_wr], RX_FIFO_HI_WATERMARK,
		//       NULL) == 0) ? RX_FIFO_HI_WATERMARK : 0;

		// Make sure that we're in RX mode.
		trans_state_ = TRANS_STATE_RX_MODE;

	}
	else if (device_status_.int_status_2.valid_preamble_detected)
	{
		// Valid preamble detected

		// Sync word detected
		// packet_start_ticks_ = PIOS_Thread_Systime();
		// if (rfm22b_dev->packet_start_ticks == 0) {
		// 	rfm22b_dev->packet_start_ticks = 1;
		// }

		// We detected the preamble, now wait for sync.
		trans_state_ = TRANS_STATE_RX_WAIT_SYNC;

	}
	else if (device_status_.int_status_2.sync_word_detected)
	{

		// read the 10-bit signed afc correction value
		uint8_t afc_corr1 = 0;
		uint8_t afc_corr0 = 0;

		// bits 9 to 2
		ret &= read_reg(AFC_CORRECTION, 	&afc_corr1);

		// bits 1 & 0
		ret &= read_reg(OOK_COUNTER_VAL_1, 	&afc_corr0);

		uint16_t afc_correction = (uint16_t)(afc_corr1 << 8 | ((afc_corr0 >> 6) & 0b11)); 

		// convert the afc value to Hz
		int32_t afc_corr = (int32_t) (frequency_step_size_ * afc_correction + 0.5f);
		afc_correction_hz_ =  (afc_corr < -127) ? -127 : ((afc_corr > 127) ? 127 : afc_corr);

		// read rx signal strength .. 45 = -100dBm, 205 = -20dBm
		uint8_t rssi = 0;
		ret &= get_rssi(&rssi);

		// convert to dBm
		rssi_dBm_ = (int8_t) (rssi >> 1) - 122;

		// Indicate that we're in RX mode.
		trans_state_ = TRANS_STATE_RX_MODE;

	}
	else if ((trans_state_ == TRANS_STATE_RX_WAIT_SYNC) && !device_status_.int_status_2.valid_preamble_detected)
	{
		// Waiting for the preamble timed out.
		rx_failure();
		return false; // int failure
	}

	return ret;
}

bool Rfm22b::rx_failure(void)
{
	bool ret = true;

	//TODO: do it...

	return ret;
}

__attribute__((interrupt))
void exti2_isr(void)
{
    handlers_->irq_handler();
}

void Rfm22b::irq_handler(void)
{

	// Disable interruption
    isr_enable(false);

	exti_reset_request(EXTI2);

	uint8_t interrupt_1 = 0;
    read_reg(INTERRUPT_STAT_1, &interrupt_1);
    if (!((interrupt_1 >> 1) & 1)) // Valid Packet Received Mask
    {
       	// Enable interruption
    	isr_enable();

    	// Nothing valid received
    	return;
    }

	receive();

    return;
}