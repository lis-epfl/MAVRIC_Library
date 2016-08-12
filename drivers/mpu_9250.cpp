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
 * \file Mpu_9250.cpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief This file is the driver for the integrated 3axis gyroscope and
 * accelerometer Mpu 9250
 *
 ******************************************************************************/
#include "drivers/mpu_9250.hpp"
#include "hal/common/time_keeper.hpp"
#include "hal/stm32/spi_stm32.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

uint8_t tt = 0;
Mpu_9250::Mpu_9250(Spi& spi, const mpu_9250_conf_t config):
    spi_(spi),
    acc_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    gyro_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    mag_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    temperature_(0.0f),
    last_update_us_(0.0f),
    config_(config)
{}

bool Mpu_9250::init(void)
{
    bool success = true;

    success &= mpu_reset();

    // Read WhoAmI for Acc and Gyr
    uint8_t whoami_command[2] = {MPU9250_WHOAMI_REG, 0};
    uint8_t whoami_id[2]      = {0};
    success &= read_reg(whoami_command, whoami_id);
    success &= whoami_id[1] == MPU9250_WHOAMI_ID ? true : false;

    uint8_t power_management[2] = {MPU9250_PWR_MGMT_REG, MPU9250_PWRMGMT_PLL_X_CLK};
    success &= write_reg(power_management);

    // Enable communication with Mag
    // uint8_t mag_com_en[] = {MPU9250_USER_CTRL_REG, (uint8_t)(MPU9250_USERCTL_I2C_MST_EN |
    //                                                          MPU9250_USERCTL_DIS_I2C)};
    // success &= write_reg(mag_com_en,2);
    // success &= mag_reset();

    // // Read WhoAmI for Mag
    // success &= mag_read_reg(AK8963_WHOAMI_REG, whoami_id);
    // success &= whoami_id[1] == AK8963_WHOAMI_ID ? true : false;

    if (!success)
    {
        return success;
    }

    /*Configuring Accelerometer and Gyroscope*/
    success  = set_acc_lpf();
    // success &= set_gyro_lpf();

    success &= set_mpu_sample_rate();

    success &= set_acc_range();
    // success &= set_gyro_range();

    /*Configuring Magnetometer*/
    // success &= set_mag_sample_rate(); // bug

    // uint8_t slv0_reg_reg[]  = {MPU9250_SLV0_REG_REG , AK8963_ST1_REG};
    // uint8_t slv0_addr_reg[] = {MPU9250_SLV0_ADDR_REG, (uint8_t)(MPU9250_AK8963_ADDR | MPU9250_READ_FLAG)};
    // uint8_t slv0_ctrl_reg[] = {MPU9250_SLV0_CTRL_REG, (uint8_t)(MPU9250_I2CSLV_EN | 8)};

    // success &= write_reg(slv0_reg_reg);
    // success &= write_reg(slv0_addr_reg);
    // success &= write_reg(slv0_ctrl_reg);

    uint8_t out_test[] = {MPU9250_ACCEL_CFG2_REG, 0};
    uint8_t in_test[] = {0,0};
    read_reg(out_test, in_test);
    success &= in_test[1] == config_.acc_filter ? true : false;

    // out_test[0] = MPU9250_SMPLRT_DIV_REG;
    // read_reg(out_test, in_test);
    // success &= in_test[1] == (uint8_t)0x09 ? true : false;

    out_test[0] = MPU9250_ACCEL_CFG_REG;
    read_reg(out_test, in_test);
    success &= in_test[1] == config_.acc_range ? true : false;


    return success;
}


bool Mpu_9250::update_acc(void)
{
    bool success = true;

    uint8_t acc_reg[7]    = {MPU9250_ACCEL_X_OUT_MSB, 0, 0, 0, 0, 0 ,0};
    uint8_t acc_buffer[7] = {0};

    success &= read_reg(acc_reg, acc_buffer, 7);

    acc_data_[0] = (float)((int16_t)(acc_buffer[1] << 8 | acc_buffer[2]));
    acc_data_[1] = (float)((int16_t)(acc_buffer[3] << 8 | acc_buffer[4]));
    acc_data_[2] = (float)((int16_t)(acc_buffer[5] << 8 | acc_buffer[6]));

    last_update_us_ = time_keeper_get_us();

    return success;
}


bool Mpu_9250::update_gyr(void)
{
    bool success = true;

    // uint8_t gyro_reg[7]    = {MPU9250_GYRO_X_OUT_MSB, 0, 0, 0, 0, 0 ,0};
    // uint8_t gyro_buffer[7] = {0};
    //
    // success &= read_reg(gyro_reg, gyro_buffer, 7);
    //
    // acc_data_[0] = (float)((int16_t)(gyro_buffer[1] << 8 | gyro_buffer[2]));
    // acc_data_[1] = (float)((int16_t)(gyro_buffer[3] << 8 | gyro_buffer[4]));
    // acc_data_[2] = (float)((int16_t)(gyro_buffer[5] << 8 | gyro_buffer[6]));
    //
    // last_update_us_ = time_keeper_get_us();

    return success;
}

bool Mpu_9250::update_mag(void)
{
    bool success = true;

    // uint8_t mag_reg[8]    = {MPU9250_GYRO_Z_OUT_LSB, 0, 0, 0, 0, 0 ,0, 0};
    // uint8_t mag_buffer[8] = {0};
    //
    // success &= read_reg(mag_reg, mag_buffer, 8);
    //
    // mag_data_[0] = (float)((int16_t)(mag_buffer[2] << 8 | mag_buffer[3]));
    // mag_data_[1] = (float)((int16_t)(mag_buffer[4] << 8 | mag_buffer[5]));
    // mag_data_[2] = (float)((int16_t)(mag_buffer[6] << 8 | mag_buffer[7]));
    //
    // last_update_us_ = time_keeper_get_us();

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

bool Mpu_9250::mpu_reset(void)
{
    bool ret = true;

    // Write reset
    uint8_t reset_command[2] = {MPU9250_USER_CTRL_REG ,MPU9250_PWRMGMT_IMU_RST};

    ret &= write_reg(reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}

bool Mpu_9250::mag_reset(void)
{
    bool ret = true;

    // Write reset
     uint8_t reset_command[2]  = {AK8963_CNTL2_REG, AK8963_CNTL2_SRST};
     ret &= mag_write_reg(reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}

bool Mpu_9250::mag_read_reg(uint8_t reg, uint8_t* in_buffer)
{
    bool ret = true;
    uint8_t status = 0;
    uint32_t timeout = 0;

    uint8_t reg_to_read[]   = {MPU9250_SLV4_REG_REG, reg};
    uint8_t out_buffer2[]   = {MPU9250_SLV4_ADDR_REG, (uint8_t)(MPU9250_AK8963_ADDR | MPU9250_READ_FLAG)};
    uint8_t i2c_slave_en[]  = {MPU9250_SLV4_CTRL_REG, MPU9250_I2CSLV_EN};
    uint8_t out_buffer4[]   = {MPU9250_I2C_MST_STATUS_REG, 0};
    uint8_t out_buffer5[]   = {MPU9250_SLV4_DI_REG, 0};
    uint8_t in_buffer1[]    = {0,0};

    ret &= write_reg(reg_to_read);
    ret &= write_reg(out_buffer2);
    ret &= write_reg(i2c_slave_en);

    // wait for I2C transaction done, use simple safety
    // escape counter to prevent endless loop in case
    // MPU9250 is broken
    do {
        if (timeout++ > 50)
            return false;

        read_reg(out_buffer4, in_buffer1);
        status = in_buffer1[1];
    } while ((status & MPU9250_I2C_MST_SLV4_DONE) == 0);

    ret &= read_reg(out_buffer5, in_buffer);
    return ret;

}

bool Mpu_9250::mag_write_reg(uint8_t* out_buffer)
{
    bool ret = true;
    uint8_t status = 0;
    uint32_t timeout = 0;

    // we will use I2C SLV4 to manipulate with AK8963 control registers
    uint8_t reg_to_read[]   = {MPU9250_SLV4_REG_REG, out_buffer[0]};
    uint8_t out_buffer2[]   = {MPU9250_SLV4_ADDR_REG, (uint8_t)(MPU9250_AK8963_ADDR & MPU9250_WRITE_FLAG)};
    uint8_t out_buffer3[]   = {MPU9250_SLV4_DO_REG, out_buffer[1]};
    uint8_t i2c_slave_en[]  = {MPU9250_SLV4_CTRL_REG, MPU9250_I2CSLV_EN};

    uint8_t out_buffer4[]   = {MPU9250_I2C_MST_STATUS_REG, 0};
    uint8_t in_buffer[]    = {0,0};

    ret &= write_reg(reg_to_read);
    ret &= write_reg(out_buffer2);
    ret &= write_reg(out_buffer3);
    ret &= write_reg(i2c_slave_en);

    // wait for I2C transaction done, use simple safety
    // escape counter to prevent endless loop in case
    // MPU9250 is broken
    do {
        if (timeout++ > 50)
            return false;

        read_reg(out_buffer4, in_buffer);
        status = in_buffer[1];
    } while ((status & MPU9250_I2C_MST_SLV4_DONE) == 0);

    if (status & MPU9250_I2C_MST_SLV4_NACK)
        return false;

    return ret;
}

bool Mpu_9250::write_reg(uint8_t* out_buffer, uint32_t nbytes)
{
    out_buffer[0] &= MPU9250_WRITE_FLAG;//(uint8_t)(MPU9250_WRITE_FLAG & out_buffer[0]);
    return spi_.transfer(out_buffer, NULL, nbytes);//sizeof(out_buffer));
}

bool Mpu_9250::read_reg(uint8_t* out_buffer, uint8_t* in_buffer, uint32_t nbytes)
{
    out_buffer[0] |= MPU9250_READ_FLAG;//(uint8_t)(MPU9250_READ_FLAG | out_buffer[0]);
    return spi_.transfer(out_buffer, in_buffer, nbytes);//sizeof(out_buffer));
}

bool Mpu_9250::set_acc_lpf(void)
{
    uint8_t out_buffer[] = {MPU9250_ACCEL_CFG2_REG, config_.acc_filter};
    return write_reg(out_buffer);
}

bool Mpu_9250::set_gyro_lpf(void)
{
    uint8_t out_buffer[] = {MPU9250_DLPF_CFG_REG, config_.gyro_filter};
    return write_reg(out_buffer);
}

bool Mpu_9250::set_mag_sample_rate(void)
{
    // uint8_t out_buffer[] = {AK8963_CNTL1_REG, AK8963_MODE_CONTINUOUS_FAST_16B};
    // return mag_write_reg(out_buffer);
    bool ret = true;

    // Write reset
     uint8_t reset_command[2]  = {AK8963_CNTL2_REG, AK8963_CNTL2_SRST};
     ret &= mag_write_reg(reset_command);

    // Let the sensor reset
    time_keeper_delay_ms(50);

    return ret;
}

bool Mpu_9250::set_mpu_sample_rate(void)
{
    uint16_t samplerate_hz      = config_.default_sample_rate;

    // mpu9250 ODR divider is unable to run from 8kHz clock like mpu60x0 :(
    // check if someone want to use 250Hz DLPF and don't want 8kHz sampling
    // and politely refuse him
    if ((config_.gyro_filter == MPU9250_GYRO_LOWPASS_250_HZ) && (samplerate_hz != 8000))
        return false;

    uint16_t filter_frequency   = 1000; // Hz

    // limit samplerate to filter frequency
    if (samplerate_hz > filter_frequency)
        samplerate_hz = filter_frequency;

    // calculate divisor, round to nearest integeter
    int32_t divisor = (int32_t)(((float)filter_frequency / samplerate_hz) + 0.5f) - 1;

    // limit resulting divisor to register value range
    if (divisor < 0)
        divisor = 0; // samplerate = 1 kHz

    if (divisor > 0xff)
        divisor = 0xff; // samplerate = 3.91 Hz

    uint8_t out_buffer[] = {MPU9250_SMPLRT_DIV_REG, (uint8_t)divisor};
    return write_reg(out_buffer);
}

bool Mpu_9250::set_acc_range(void)
{
    uint8_t out_buffer[] = {MPU9250_ACCEL_CFG_REG, config_.acc_range};
    return write_reg(out_buffer);
}

bool Mpu_9250::set_gyro_range(void)
{
    uint8_t out_buffer[] = {MPU9250_GYRO_CFG_REG, config_.gyro_range};
    return write_reg(out_buffer);
}
