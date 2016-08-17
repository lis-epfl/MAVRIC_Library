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
 * \file mpu9250.hpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief This file is the driver for the integrated 3axis gyroscope and
 * accelerometer MPU 9250
 *
 ******************************************************************************/


#ifndef MPU_9250_HPP_
#define MPU_9250_HPP_

#include <cstdint>
#include <array>
#include "drivers/accelerometer.hpp"
#include "drivers/gyroscope.hpp"
#include "drivers/magnetometer.hpp"
#include "hal/common/spi.hpp"

typedef enum
{
    MPU9250_ACC_2G    = 0x00,
    MPU9250_ACC_4G    = 0x08,
    MPU9250_ACC_8G    = 0x10,
    MPU9250_ACC_16G   = 0x18,
} mpu_9250_acc_range_t ;

typedef enum
{
    MPU9250_GYRO_250_DEG  = 0x00,
    MPU9250_GYRO_500_DEG  = 0x08,
    MPU9250_GYRO_1000_DEG = 0x10,
    MPU9250_GYRO_2000_DEG = 0x18,
} mpu_9250_gyro_range_t ;

typedef enum
{
    MPU9250_GYRO_LOWPASS_250_HZ = 0x00,
    MPU9250_GYRO_LOWPASS_184_HZ = 0x01,
    MPU9250_GYRO_LOWPASS_92_HZ  = 0x02,
    MPU9250_GYRO_LOWPASS_41_HZ  = 0x03,
    MPU9250_GYRO_LOWPASS_20_HZ  = 0x04,
    MPU9250_GYRO_LOWPASS_10_HZ  = 0x05,
    MPU9250_GYRO_LOWPASS_5_HZ   = 0x06,
} mpu_9250_gyro_filter_t;

typedef enum
{
    MPU9250_ACC_LOWPASS_460_HZ = 0x00,
    MPU9250_ACC_LOWPASS_184_HZ = 0x01,
    MPU9250_ACC_LOWPASS_92_HZ  = 0x02,
    MPU9250_ACC_LOWPASS_41_HZ  = 0x03,
    MPU9250_ACC_LOWPASS_20_HZ  = 0x04,
    MPU9250_ACC_LOWPASS_10_HZ  = 0x05,
    MPU9250_ACC_LOWPASS_5_HZ   = 0x06,
} mpu_9250_acc_filter_t;

/**
 * \brief   Configuration structure for mpu 9250
 */
typedef struct
{
    mpu_9250_acc_filter_t       acc_filter;         ///< Accelerometer lp filter cut off freq
    mpu_9250_acc_range_t        acc_range;
    mpu_9250_gyro_filter_t      gyro_filter;        ///< Gyroscope lp filter cut off freq
    mpu_9250_gyro_range_t       gyro_range;
    uint16_t                    default_sample_rate;///< Default sample rate in Herz

} mpu_9250_conf_t;

/**
 * \brief   Default configuration for mpu 9250
 *
 * \return  Conf structure
 */
static inline mpu_9250_conf_t mpu_9250_default_config()
{
    mpu_9250_conf_t conf    = {};

    conf.acc_filter             = MPU9250_ACC_LOWPASS_184_HZ;
    conf.acc_range              = MPU9250_ACC_2G;
    conf.gyro_filter            = MPU9250_GYRO_LOWPASS_184_HZ;
    conf.gyro_range             = MPU9250_GYRO_500_DEG;
    conf.default_sample_rate    = 500;

    return conf;
};

/**
 * \brief       Intermediate interface class for the accelero inside MPU 9250
 *
 * \details     Inherits only accelerometer
 */
class Mpu_9250_acc: public Accelerometer
{
public:
    virtual bool update_acc(void) = 0;

    bool update(void)
    {
        return update_acc();
    }
};


/**
 * \brief       Intermediate interface class for the gyro inside MPU 9250
 *
 * \details     Inherits only gyroscope
 */
class Mpu_9250_gyr: public Gyroscope
{
public:
    virtual bool update_gyr(void) = 0;

    bool update(void)
    {
        return update_gyr();
    }
};

/**
 * \brief       Intermediate interface class for the mag inside MPU 9250
 *
 * \details     Inherits only magnetometer
 */
 class Mpu_9250_mag: public Magnetometer
{
public:
    virtual bool update_mag(void) = 0;

    bool update(void)
    {
        return update_mag();
    }
};


/**
 * \brief       Driver for sensor MPU 9250
 *
 * \details     This sensor is at the same time a accelerometer and a gyroscope
 *              The inherited method Accelerometer::update is implemented as Lsm330dlc::update_acc
 *              The inherited method Gyroscope::update is implemented as Lsm330dlc::update_gyr
 */
class Mpu_9250: public Mpu_9250_acc, public Mpu_9250_gyr, public Mpu_9250_mag
{
public:
    /**
     * \brief   Constructor
     *
     * \param   spi     Reference to SPI device
     */
    Mpu_9250(Spi& spi, const mpu_9250_conf_t config = mpu_9250_default_config());


    /**
     * \brief   Initialise the sensor
     * \details Sends configuration via SPI, the SPI peripheral must be
     *          activated before this method is called
     *
     * \return  true    Success
     * \return  false   Failed
     */
    bool init(void);


    /**
     * \brief   Main update function
     * \details Get new data from the sensor
     *
     * \return  true    Success
     * \return  false   Failed
     */
    bool update_acc(void);
    bool update_gyr(void);
    bool update_mag(void);


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Update time
     */
    const float& last_update_us(void) const;


    /**
     * \brief   Get X, Y and Z components of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const std::array<float, 3>& gyro(void) const;



    /**
     * \brief   Get X component of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& gyro_X(void) const;


    /**
     * \brief   Get Y component of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& gyro_Y(void) const;


    /**
     * \brief   Get Z component of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& gyro_Z(void) const;


    /**
     * \brief   Get X, Y and Z components of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const std::array<float, 3>& acc(void) const;


    /**
     * \brief   Get X component of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& acc_X(void) const;


    /**
     * \brief   Get Y component of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& acc_Y(void) const;


    /**
     * \brief   Get Z component of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& acc_Z(void) const;

    /**
     * \brief   Get X, Y and Z components of magnetic field
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const std::array<float, 3>& mag(void) const;


    /**
     * \brief   Get X component of magnetic field
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& mag_X(void) const;


    /**
     * \brief   Get Y component of magnetic field
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& mag_Y(void) const;


    /**
     * \brief   Get Z component of magnetic field
     *
     * \detail  This is raw data, so X, Y and Z components are biased, not scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal, scaling and axis rotations
     *
     * \return  Value
     */
    const float& mag_Z(void) const;

    /**
     * \brief   Get sensor temperature
     *
     * \return  Value
     */
    const float& temperature(void) const;

    /**
     * \brief   Reset acc and gyro
     *
     * \return  success
     */
    bool mpu_reset(void);

    /**
     * \brief   Reset mag
     *
     * \return  success
     */
    bool mag_reset(void);

    /**
     * \brief   Read register from magnetometer
     *
     * \param   reg         register to read
     * \param   in_buffer   buffer with read values
     *
     * \return  success
     */
    bool mag_read_reg(uint8_t reg, uint8_t* in_buffer);

    /**
     * \brief   Write values in register of magnetometer
     *
     * \param   out_buffer   buffer with register to read and values to write
     *
     * \return  success
     */
    bool mag_write_reg(uint8_t* out_buffer);


    bool write_reg(uint8_t* out_buffer, uint32_t nbytes = 2);
    bool read_reg(uint8_t* out_buffer, uint8_t* in_buffer, uint32_t nbytes = 2);


    /**
     * Register adress and values
     */

    const uint8_t AK8963_WHOAMI_REG     = 0x00;
    const uint8_t AK8963_ST1_REG        = 0x02;
    const uint8_t AK8963_HXL            = 0x03;
    const uint8_t AK8963_ST2_REG        = 0x09;
    const uint8_t AK8963_WHOAMI_ID      = 0x48;

    const uint8_t MPU9250_WRITE_FLAG = 0x7f;
    const uint8_t MPU9250_READ_FLAG  = 0x80;

    const uint8_t MPU9250_WHOAMI_ID    = 0x71;

    const uint8_t AK8963_CNTL1_REG                  = 0x0A;
    const uint8_t AK8963_CNTL2_REG                  = 0x0B;
    const uint8_t AK8963_CNTL2_SRST                 = 0x01;
    const uint8_t AK8963_MODE_CONTINUOUS_NRML_16B   = 0x12;
    const uint8_t AK8963_MODE_CONTINUOUS_FAST_16B   = 0x16;
    const uint8_t MPU9250_AK8963_ADDR               = 0x0C;

    /* MPU9250 Addresses */
    const uint8_t MPU9250_SMPLRT_DIV_REG        = 0x19;
    const uint8_t MPU9250_DLPF_CFG_REG          = 0x1A;
    const uint8_t MPU9250_GYRO_CFG_REG          = 0x1B;
    const uint8_t MPU9250_ACCEL_CFG_REG         = 0x1C;
    const uint8_t MPU9250_ACCEL_CFG2_REG        = 0x1D;
    const uint8_t MPU9250_SLV0_ADDR_REG         = 0x25;
    const uint8_t MPU9250_SLV0_REG_REG          = 0x26;
    const uint8_t MPU9250_SLV0_CTRL_REG         = 0x27;
    const uint8_t MPU9250_SLV4_ADDR_REG         = 0x31;
    const uint8_t MPU9250_SLV4_REG_REG          = 0x32;
    const uint8_t MPU9250_SLV4_DO_REG           = 0x33;
    const uint8_t MPU9250_SLV4_CTRL_REG         = 0x34;
    const uint8_t MPU9250_SLV4_DI_REG           = 0x35;
    const uint8_t MPU9250_I2C_MST_STATUS_REG    = 0x36;
    const uint8_t MPU9250_ACCEL_X_OUT_MSB       = 0x3B;
    const uint8_t MPU9250_ACCEL_X_OUT_LSB       = 0x3C;
    const uint8_t MPU9250_ACCEL_Y_OUT_MSB       = 0x3D;
    const uint8_t MPU9250_ACCEL_Y_OUT_LSB       = 0x3E;
    const uint8_t MPU9250_ACCEL_Z_OUT_MSB       = 0x3F;
    const uint8_t MPU9250_ACCEL_Z_OUT_LSB       = 0x40;
    const uint8_t MPU9250_GYRO_X_OUT_MSB        = 0x43;
    const uint8_t MPU9250_GYRO_X_OUT_LSB        = 0x44;
    const uint8_t MPU9250_GYRO_Y_OUT_MSB        = 0x45;
    const uint8_t MPU9250_GYRO_Y_OUT_LSB        = 0x46;
    const uint8_t MPU9250_GYRO_Z_OUT_MSB        = 0x47;
    const uint8_t MPU9250_GYRO_Z_OUT_LSB        = 0x48;
    const uint8_t MPU9250_EXT_SENS_DATA_00      = 0x49;
    const uint8_t MPU9250_USER_CTRL_REG         = 0x6A;
    const uint8_t MPU9250_PWR_MGMT_REG          = 0x6B;
    const uint8_t MPU9250_WHOAMI_REG            = 0x75;

    /* I2C master status register bits */
    const uint8_t MPU9250_I2C_MST_SLV4_NACK = 0x10;
    const uint8_t MPU9250_I2C_MST_SLV4_DONE = 0x40;

    /* I2C SLV register bits */
    const uint8_t MPU9250_I2CSLV_EN     = 0x80;

    /* Power management and clock selection */
    const uint8_t MPU9250_PWRMGMT_IMU_RST   = 0x80;
    const uint8_t MPU9250_PWRMGMT_PLL_X_CLK = 0x01;

    /* User control registers */
    const uint8_t MPU9250_USERCTL_DIS_I2C       = 0x10;
    const uint8_t MPU9250_USERCTL_I2C_MST_EN    = 0x20;
    const uint8_t MPU9250_USERCTL_GYRO_RST      = 0x01;

private:
    Spi&                 spi_;              ///< SPI peripheral
    std::array<float, 3> acc_data_;         ///< Accelerometer data
    std::array<float, 3> gyro_data_;        ///< Gyroscope data
    std::array<float, 3> mag_data_;         ///< Magnetometer data
    float                temperature_;      ///< Temperature
    float                last_update_us_;   ///< Last udate time in microseconds
    mpu_9250_conf_t      config_;

    /**
     * \brief   Set accelerometer lowpass filter cut-off frequency
     *
     * \return  true if success
     */
    bool set_acc_lpf(void);

    /**
     * \brief   Set gyrometer lowpass filter cut-off frequency
     *
     * \return  true if success
     */
    bool set_gyro_lpf(void);

    /**
     * \brief   Set sampling frequency of accs and gyro axes
     *
     * \return  true if success
     */
    bool set_mpu_sample_rate(void);

    /**
     * \brief   Set sampling frequency of mag axes
     *
     * \return  true if success
     */
    bool set_mag_sample_rate(void);

    bool set_acc_range(void);
    bool set_gyro_range(void);

};

#endif /* MPU_9250_HPP_ */
