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
 * \file mpu_9250.hpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief This file is the driver for the integrated 3axis gyroscope,
 *        accelerometer and magnetometer: MPU 9250
 *
 ******************************************************************************/


#ifndef MPU_9250_HPP_
#define MPU_9250_HPP_

#include <cstdint>
#include <array>

#include "drivers/accelerometer.hpp"
#include "drivers/gyroscope.hpp"
#include "drivers/magnetometer.hpp"

#include "hal/common/gpio.hpp"
#include "hal/common/spi.hpp"

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
 * \details     This sensor is at the same time a accelerometer and a gyroscope and a magnetometer
 *              The inherited method Accelerometer::update is implemented as Mpu9250::update_acc
 *              The inherited method Gyroscope::update is implemented as Mpu9250::update_gyr
 *              The inherited method Magnetometer::update is implemented as Mpu9250::update_mag
 */
class Mpu_9250: public Mpu_9250_acc, public Mpu_9250_gyr, public Mpu_9250_mag
{
public:
    /*
     * \brief   Enum for defining lowpass filter frequency
     *          for accelerometer
     */
    typedef enum
    {
        ACC_LOWPASS_460_HZ = 0x00,
        ACC_LOWPASS_184_HZ = 0x01,
        ACC_LOWPASS_92_HZ  = 0x02,
        ACC_LOWPASS_41_HZ  = 0x03,
        ACC_LOWPASS_20_HZ  = 0x04,
        ACC_LOWPASS_10_HZ  = 0x05,
        ACC_LOWPASS_5_HZ   = 0x06,
    } acc_filter_t;

    /*
     * \brief   Enum for defining range for accelerometer
     */
    typedef enum
    {
        ACC_2G    = 0x00,
        ACC_4G    = 0x08,
        ACC_8G    = 0x10,
        ACC_16G   = 0x18,
    } acc_range_t ;

    /*
     * \brief   Enum for defining lowpass filter frequency
     *          for gyroscope
     */
    typedef enum
    {
        GYRO_LOWPASS_250_HZ = 0x00,
        GYRO_LOWPASS_184_HZ = 0x01,
        GYRO_LOWPASS_92_HZ  = 0x02,
        GYRO_LOWPASS_41_HZ  = 0x03,
        GYRO_LOWPASS_20_HZ  = 0x04,
        GYRO_LOWPASS_10_HZ  = 0x05,
        GYRO_LOWPASS_5_HZ   = 0x06,
    } gyro_filter_t;

    /*
     * \brief   Enum for defining range for gyroscope
     */
    typedef enum
    {
        GYRO_250_DEG  = 0x00,
        GYRO_500_DEG  = 0x08,
        GYRO_1000_DEG = 0x10,
        GYRO_2000_DEG = 0x18,
    } gyro_range_t ;

    /**
     * \brief   Configuration structure for mpu 9250
     */
    typedef struct
    {
        acc_filter_t       acc_filter;         ///< Accelerometer lp filter cut off freq
        acc_range_t        acc_range;          ///< Accelerometer range
        gyro_filter_t      gyro_filter;        ///< Gyroscope lp filter cut off freq
        gyro_range_t       gyro_range;         ///< Gyroscope range
        uint16_t           default_sample_rate;///< Default sample rate in Herz

    } conf_t;

    /**
     * \brief   Default configuration for mpu 9250
     *
     * \return  Conf structure
     */
    static inline conf_t mpu_9250_default_config()
    {
        conf_t conf                 = {};

        conf.acc_filter             = ACC_LOWPASS_184_HZ;
        conf.acc_range              = ACC_2G;
        conf.gyro_filter            = GYRO_LOWPASS_184_HZ;
        conf.gyro_range             = GYRO_500_DEG;
        conf.default_sample_rate    = 500; // in Hz

        return conf;
    };

    /**
     * \brief   Constructor
     *
     * \param   spi         Reference to SPI device
     * \param   nss_gpio    Reference to Slave Select GPIO
     * \param   config      Device configuration
     */
    Mpu_9250(Spi& spi, Gpio& nss_gpio, const conf_t config = mpu_9250_default_config());


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
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const std::array<float, 3>& gyro(void) const;



    /**
     * \brief   Get X component of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const float& gyro_X(void) const;


    /**
     * \brief   Get Y component of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const float& gyro_Y(void) const;


    /**
     * \brief   Get Z component of angular velocity
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const float& gyro_Z(void) const;


    /**
     * \brief   Get X, Y and Z components of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const std::array<float, 3>& acc(void) const;


    /**
     * \brief   Get X component of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const float& acc_X(void) const;


    /**
     * \brief   Get Y component of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
     *
     * \return  Value
     */
    const float& acc_Y(void) const;


    /**
     * \brief   Get Z component of acceleration
     *
     * \detail  This is raw data, so X, Y and Z components are biased, are scaled,
     *          and given in the sensor frame (not in the UAV frame).
     *          Use an Imu object to handle bias removal and axis rotations
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
     * \brief   Reset magnetometer
     *
     * \return  success
     */
    bool mag_reset(void);

    /**
     * \brief   Read register from magnetometer
     *
     * \param   reg         Register to read from
     * \param   in_data     Incoming data
     *
     * \return  success
     */
    bool mag_read_reg(uint8_t reg, uint8_t* in_data);

    /**
     * \brief   Write values in register of ak8963
     *
     * \param   reg         Register to write to
     * \param   out_data    Outgoing data
     *
     * \return  success
     */
    bool mag_write_reg(uint8_t reg, uint8_t* out_data);

    /**
     * \brief   Write register of mpu 9250
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
     * \brief   Read register of mpu 9250

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

    // AK8963 register addresses
    static const uint8_t AK8963_WHOAMI_REG     = 0x00;
    static const uint8_t AK8963_ST1_REG        = 0x02;
    static const uint8_t AK8963_HXL            = 0x03;
    static const uint8_t AK8963_ST2_REG        = 0x09;

    // AK8963 register bits
    static const uint8_t AK8963_CNTL1_CONT_8HZ     = 0x02;
    static const uint8_t AK8963_CNTL1_CONT_100HZ   = 0x06;
    static const uint8_t AK8963_CNTL1_16BITS       = 0x10;
    static const uint8_t AK8963_CNTL1_REG          = 0x0A;
    static const uint8_t AK8963_CNTL2_REG          = 0x0B;
    static const uint8_t AK8963_CNTL2_SRST         = 0x01;
    static const uint8_t AK8963_WHOAMI_ID          = 0x48;

    // MPU9250 register adresses
    static const uint8_t AK8963_ADDR           = 0x0C;
    static const uint8_t SMPLRT_DIV_REG        = 0x19;
    static const uint8_t DLPF_CFG_REG          = 0x1A;
    static const uint8_t GYRO_CFG_REG          = 0x1B;
    static const uint8_t ACCEL_CFG_REG         = 0x1C;
    static const uint8_t ACCEL_CFG2_REG        = 0x1D;
    static const uint8_t SLV0_ADDR_REG         = 0x25;
    static const uint8_t SLV0_REG_REG          = 0x26;
    static const uint8_t SLV0_CTRL_REG         = 0x27;
    static const uint8_t SLV4_ADDR_REG         = 0x31;
    static const uint8_t SLV4_REG_REG          = 0x32;
    static const uint8_t SLV4_DO_REG           = 0x33;
    static const uint8_t SLV4_CTRL_REG         = 0x34;
    static const uint8_t SLV4_DI_REG           = 0x35;
    static const uint8_t I2C_MST_STATUS_REG    = 0x36;
    static const uint8_t ACCEL_X_OUT_MSB       = 0x3B;
    static const uint8_t ACCEL_X_OUT_LSB       = 0x3C;
    static const uint8_t ACCEL_Y_OUT_MSB       = 0x3D;
    static const uint8_t ACCEL_Y_OUT_LSB       = 0x3E;
    static const uint8_t ACCEL_Z_OUT_MSB       = 0x3F;
    static const uint8_t ACCEL_Z_OUT_LSB       = 0x40;
    static const uint8_t GYRO_X_OUT_MSB        = 0x43;
    static const uint8_t GYRO_X_OUT_LSB        = 0x44;
    static const uint8_t GYRO_Y_OUT_MSB        = 0x45;
    static const uint8_t GYRO_Y_OUT_LSB        = 0x46;
    static const uint8_t GYRO_Z_OUT_MSB        = 0x47;
    static const uint8_t GYRO_Z_OUT_LSB        = 0x48;
    static const uint8_t EXT_SENS_DATA_00      = 0x49;
    static const uint8_t USER_CTRL_REG         = 0x6A;
    static const uint8_t PWR_MGMT_REG          = 0x6B;
    static const uint8_t WHOAMI_REG            = 0x75;

    // MPU9250 register bits
    static const uint8_t READ_FLAG     = 0x80;
    static const uint8_t WRITE_FLAG    = 0x7f;
    static const uint8_t WHOAMI_ID     = 0x71;

    // I2C master status register bits
    static const uint8_t I2C_MST_SLV4_NACK = 0x10;
    static const uint8_t I2C_MST_SLV4_DONE = 0x40;

    // I2C SLV register bits
    static const uint8_t I2CSLV_EN     = 0x80;

    // Power management and clock selection
    static const uint8_t PWRMGMT_IMU_RST   = 0x80;
    static const uint8_t PWRMGMT_PLL_X_CLK = 0x01;

    // User control registers
    static const uint8_t USERCTL_DIS_I2C       = 0x10;
    static const uint8_t USERCTL_I2C_MST_EN    = 0x20;
    static const uint8_t USERCTL_GYRO_RST      = 0x01;

private:
    Spi&                    spi_;              ///< SPI peripheral
    Gpio&                   nss_;              ///< Slave Select GPIO
    std::array<float, 3>    acc_data_;         ///< Accelerometer data
    std::array<float, 3>    gyro_data_;        ///< Gyroscope data
    std::array<float, 3>    mag_data_;         ///< Magnetometer data
    float                   temperature_;      ///< Temperature
    float                   last_update_us_;   ///< Last udate time in microseconds
    conf_t                  config_;           ///< Device configuration
    float                   acc_scale_;        ///< Acceleromter data scaling factor
    float                   gyro_scale_;       ///< Gyroscope data scaling factor

    /**
     * \brief   Set accelerometer lowpass filter cut-off frequency
     *
     * \return  success
     */
    bool set_acc_lpf(void);

    /**
     * \brief   Set gyrometer lowpass filter cut-off frequency
     *
     * \return  success
     */
    bool set_gyro_lpf(void);

    /**
     * \brief   Set sampling frequency of acc and gyro axes
     *
     * \return  success
     */
    bool set_mpu_sample_rate(void);

    /**
     * \brief   Set operating mode of mag
     *
     * \detail  Set the mode of measurments (single,
     *          continuous, ...) and the format of the
     *          raw data (14, 16 bits)
     *
     * \return  success
     */
    bool set_mag_mode(void);

    /**
     * \brief   Set operating range of accelerometer
     *
     * \return  success
     */
    bool set_acc_range(void);

    /**
     * \brief   Set operating range of gyroscope
     *
     * \return  success
     */
    bool set_gyro_range(void);

    /**
     * \brief   Select Slave
     */
    void select_slave(void);

    /**
     * \brief   Unselect Slave
     */
    void unselect_slave(void);

};

#endif /* MPU_9250_HPP_ */
