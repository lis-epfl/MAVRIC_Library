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
 * \file mpu_9250.cpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief This file is the driver for the integrated 3axis gyroscope,
 *        accelerometer and magnetometer: MPU 9250
 *
 ******************************************************************************/
#include "drivers/mpu_9250.hpp"
#include "hal/common/time_keeper.hpp"
#include "hal/common/spi.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Mpu_9250::Mpu_9250(Spi& spi, Gpio& nss_gpio, const conf_t config):
    spi_(spi),
    nss_(nss_gpio),
    acc_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    gyro_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    mag_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    temperature_(0.0f),
    last_update_us_(0.0f),
    config_(config)
{
    switch (config.acc_range)
    {
        case ACC_2G:
            acc_scale_ = 16384;
            break;

        case ACC_4G:
            acc_scale_ = 8192;
            break;

        case ACC_8G:
            acc_scale_ = 4096;
            break;

        case ACC_16G:
            acc_scale_ = 2048;
            break;
    }

    switch (config.gyro_range)
    {
        case GYRO_250_DEG:
            gyro_scale_ = 131;
            break;

        case GYRO_500_DEG:
            gyro_scale_ = 65.5;
            break;

        case GYRO_1000_DEG:
            gyro_scale_ = 32.8;
            break;

        case GYRO_2000_DEG:
            gyro_scale_ = 16.4;
            break;
    }

    // converting from dps to rps
    gyro_scale_ *= 180.0f/3.14159f;
}

bool Mpu_9250::init(void)
{
    bool success = true;

    // Initializing Slave Select GPIO
    success &= nss_.init();
    unselect_slave();

    // Reset accelerometer and gyroscope
    success &= mpu_reset();

    // Read WhoAmI for Acc and Gyr
    uint8_t whoami_answer   = 0;
    success &= read_reg(WHOAMI_REG, &whoami_answer);
    success &= whoami_answer == WHOAMI_ID ? true : false;

    // Automatic best available clock selection
    uint8_t power_management_cmd = PWRMGMT_PLL_X_CLK;
    success &= write_reg(PWR_MGMT_REG, &power_management_cmd);

    // Enable communication with Mag
    uint8_t mag_communication_en = (uint8_t)(USERCTL_I2C_MST_EN |
                                             USERCTL_DIS_I2C);
    success &= write_reg(USER_CTRL_REG, &mag_communication_en);

    // Reset magnetometer
    success &= mag_reset();

    // Read WhoAmI for Mag
    success &= mag_read_reg(AK8963_WHOAMI_REG, &whoami_answer);
    success &= whoami_answer == AK8963_WHOAMI_ID ? true : false;

    if (!success)
    {
        // Returning if device not responding
        return success;
    }

    // Configuring Magnetometer, has to be done before configuring
    // acc and gyro in order to work
    success = set_mag_mode();

    // Use slv0 to access mag raw datas (x,y,z and status 2 register)
    // burst read used
    uint8_t first_reg       = AK8963_HXL;
    uint8_t address         = (uint8_t)(AK8963_ADDR | READ_FLAG);
    uint8_t slv0_ctrl_reg   = (uint8_t)(I2CSLV_EN | 7);

    success &= write_reg(SLV0_REG_REG , &first_reg);
    success &= write_reg(SLV0_ADDR_REG, &address);
    success &= write_reg(SLV0_CTRL_REG, &slv0_ctrl_reg);

    // Configuring Accelerometer and Gyroscope
    success &= set_acc_lpf();
    success &= set_gyro_lpf();
    success &= set_mpu_sample_rate();
    success &= set_acc_range();
    success &= set_gyro_range();

    return success;
}


bool Mpu_9250::update_acc(void)
{
    bool success = true;

    // Read raw data from accelerometer (big endian)
    uint8_t acc_data_raw[6] = {0};
    success &= read_reg(ACCEL_X_OUT_MSB, acc_data_raw, 6);

    acc_data_[0] = (float)((int16_t)(acc_data_raw[0] << 8 | acc_data_raw[1]));
    acc_data_[1] = (float)((int16_t)(acc_data_raw[2] << 8 | acc_data_raw[3]));
    acc_data_[2] = (float)((int16_t)(acc_data_raw[4] << 8 | acc_data_raw[5]));

    acc_data_[0] /= acc_scale_;
    acc_data_[1] /= acc_scale_;
    acc_data_[2] /= acc_scale_;

    last_update_us_ = time_keeper_get_us();

    return success;
}


bool Mpu_9250::update_gyr(void)
{
    bool success = true;

    // Read raw data from gyroscope (big endian)
    uint8_t gyro_data_raw[6] = {0};
    success &= read_reg(GYRO_X_OUT_MSB, gyro_data_raw, 6);

    gyro_data_[0] = (float)((int16_t)(gyro_data_raw[0] << 8 | gyro_data_raw[1]));
    gyro_data_[1] = (float)((int16_t)(gyro_data_raw[2] << 8 | gyro_data_raw[3]));
    gyro_data_[2] = (float)((int16_t)(gyro_data_raw[4] << 8 | gyro_data_raw[5]));

    // Gyroscope is sending dps, scaling it to get rps
    gyro_data_[0] /= gyro_scale_;
    gyro_data_[1] /= gyro_scale_;
    gyro_data_[2] /= gyro_scale_;
    
    last_update_us_ = time_keeper_get_us();

    return success;
}

bool Mpu_9250::update_mag(void)
{
    bool success = true;

    // Read raw data from magnetometer (little endian)
    uint8_t mag_data_raw[6] = {0};
    success &= read_reg(EXT_SENS_DATA_00, mag_data_raw, 6);
    
    mag_data_[0] = (float)((int16_t)(mag_data_raw[1] << 8 | mag_data_raw[0]));
    mag_data_[1] = (float)((int16_t)(mag_data_raw[3] << 8 | mag_data_raw[2]));
    mag_data_[2] = (float)((int16_t)(mag_data_raw[5] << 8 | mag_data_raw[4]));
    
    last_update_us_ = time_keeper_get_us();

    return success;
}


const float& Mpu_9250::last_update_us(void) const
{
    return last_update_us_;
}


const std::array<float, 3>& Mpu_9250::gyro(void) const
{
    return gyro_data_;
}


const float& Mpu_9250::gyro_X(void) const
{
    return gyro_data_[0];
}


const float& Mpu_9250::gyro_Y(void) const
{
    return gyro_data_[1];
}


const float& Mpu_9250::gyro_Z(void) const
{
    return gyro_data_[2];
}


const std::array<float, 3>& Mpu_9250::acc(void) const
{
    return acc_data_;
}


const float& Mpu_9250::acc_X(void) const
{
    return acc_data_[0];
}


const float& Mpu_9250::acc_Y(void) const
{
    return acc_data_[1];
}


const float& Mpu_9250::acc_Z(void) const
{
    return acc_data_[2];
}

const std::array<float, 3>& Mpu_9250::mag(void) const
{
    return mag_data_;
}


const float& Mpu_9250::mag_X(void) const
{
    return mag_data_[0];
}


const float& Mpu_9250::mag_Y(void) const
{
    return mag_data_[1];
}


const float& Mpu_9250::mag_Z(void) const
{
    return mag_data_[2];
}


const float& Mpu_9250::temperature(void) const
{
    return temperature_;
}

bool Mpu_9250::mag_reset(void)
{
    bool ret = true;

    // Write reset
     uint8_t reset_command = AK8963_CNTL2_SRST;
     ret &= mag_write_reg(AK8963_CNTL2_REG, &reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}

bool Mpu_9250::mpu_reset(void)
{
    bool ret = true;

    uint8_t reset_command = PWRMGMT_IMU_RST;

    // Write reset
    ret &= write_reg(PWR_MGMT_REG, &reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}

bool Mpu_9250::mag_read_reg(uint8_t reg, uint8_t* in_data)
{
    bool     ret     = true;
    uint8_t  status  = 0;
    uint32_t timeout = 0;

    // Use of I2C SLV4 to access AK8963 (mag) registers
    uint8_t reg_to_read     = reg;
    uint8_t address         = (uint8_t)(AK8963_ADDR | READ_FLAG);
    uint8_t i2c_slave_en    = I2CSLV_EN;

    ret &= write_reg(SLV4_REG_REG , &reg_to_read);
    ret &= write_reg(SLV4_ADDR_REG, &address);
    ret &= write_reg(SLV4_CTRL_REG, &i2c_slave_en);

    // Wait for I2C transaction done
    do {
        if (timeout++ > 50)
        {
            // Returns to prevent endless loop
            return false;
        }

        // Check if transaction done
        read_reg(I2C_MST_STATUS_REG, &status);
    } while (!(status & I2C_MST_SLV4_DONE));

    // Read data
    ret &= read_reg(SLV4_DI_REG, in_data); // TODO: allow multiple data reading

    return ret;
}

bool Mpu_9250::mag_write_reg(uint8_t reg, uint8_t* out_data)
{
    bool ret = true;
    uint8_t status = 0;
    uint32_t timeout = 0;

    // Use of I2C SLV4 to access AK8963 (mag) registers
    uint8_t reg_to_write    = reg;
    uint8_t address         = (uint8_t)(AK8963_ADDR & WRITE_FLAG);
    uint8_t data_to_write   = out_data[0];                       // TODO: allow multiple data writing
    uint8_t i2c_slave_en    = I2CSLV_EN;

    ret &= write_reg(SLV4_REG_REG , &reg_to_write);
    ret &= write_reg(SLV4_ADDR_REG, &address);
    ret &= write_reg(SLV4_DO_REG  , &data_to_write);
    ret &= write_reg(SLV4_CTRL_REG, &i2c_slave_en);

    // Wait for I2C transaction done
    do {
        if (timeout++ > 50)
        {
            // Return to prevent endless loop
            return false;
        }

        // Check if transaction done
        read_reg(I2C_MST_STATUS_REG, &status);
    } while (!(status & I2C_MST_SLV4_DONE));

    if (status & I2C_MST_SLV4_NACK)
    {
        return false;
    }

    return ret;
}

bool Mpu_9250::read_reg(uint8_t reg, uint8_t* in_data, uint32_t nbytes)
{
    bool ret = true;
    reg |= READ_FLAG;

    select_slave();

    // Write register
    ret &= spi_.transfer(&reg, NULL, 1);

    // Read data from register
    ret &= spi_.transfer(NULL, in_data, nbytes);

    unselect_slave();

    return ret;
}

bool Mpu_9250::write_reg(uint8_t reg, uint8_t* out_data, uint32_t nbytes)
{
    bool ret = true;
    reg &= WRITE_FLAG;

    select_slave();

    // Write register
    ret &= spi_.transfer(&reg, NULL, 1);

    // Write data to register
    ret &= spi_.transfer(out_data, NULL, nbytes);

    unselect_slave();


    return ret;
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
bool Mpu_9250::set_acc_lpf(void)
{
    uint8_t accelerometer_lpf = config_.acc_filter;
    return write_reg(ACCEL_CFG2_REG, &accelerometer_lpf);
}

bool Mpu_9250::set_gyro_lpf(void)
{
    uint8_t gyroscope_lpf = config_.gyro_filter;
    return write_reg(DLPF_CFG_REG, &gyroscope_lpf);
}

bool Mpu_9250::set_mag_mode(void)
{
    uint8_t mode = (uint8_t)(AK8963_CNTL1_16BITS | AK8963_CNTL1_CONT_100HZ);
    return mag_write_reg(AK8963_CNTL1_REG, &mode);
}

bool Mpu_9250::set_mpu_sample_rate(void)
{
    uint16_t samplerate_hz = config_.default_sample_rate;

    // mpu9250 ODR divider is unable to run from 8kHz clock like mpu60x0 :(
    // check if someone want to use 250Hz DLPF and don't want 8kHz sampling
    // and politely refuse him
    if ((config_.gyro_filter == GYRO_LOWPASS_250_HZ) && (samplerate_hz != 8000))
    {
        return false;
    }

    uint16_t filter_frequency   = 1000; // Hz

    // limit samplerate to filter frequency
    if (samplerate_hz > filter_frequency)
    {
        samplerate_hz = filter_frequency;
    }

    // calculate divisor, round to nearest integeter
    int32_t divisor = (int32_t)(((float)filter_frequency / samplerate_hz) + 0.5f) - 1;

    // limit resulting divisor to register value range
    if (divisor < 0)
    {
        divisor = 0; // samplerate = 1 kHz
    }

    if (divisor > 0xff)
    {
        divisor = 0xff; // samplerate = 3.91 Hz
    }

    // Write sample rate divisor in register (effective divisor is sample
    // rate divisor +1)
    uint8_t sample_rate_div = (uint8_t)divisor;
    return write_reg(SMPLRT_DIV_REG, &sample_rate_div);
}

bool Mpu_9250::set_acc_range(void)
{
    uint8_t accelerometer_range = config_.acc_range;
    return write_reg(ACCEL_CFG_REG, &accelerometer_range);
}

bool Mpu_9250::set_gyro_range(void)
{
    uint8_t gyroscope_range = config_.gyro_range;
    return write_reg(GYRO_CFG_REG, &gyroscope_range);
}

void Mpu_9250::select_slave(void)
{
    nss_.set_low();
    return;
}

void Mpu_9250::unselect_slave(void)
{
    nss_.set_high();
    return;
}
