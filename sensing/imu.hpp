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
 * \file imu.h
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *
 * \brief Inertial measurement unit (IMU)
 *
 ******************************************************************************/


#ifndef IMU_HPP_
#define IMU_HPP_

#include <stdint.h>
#include <stdbool.h>
#include <array>

#include "drivers/accelerometer.hpp"
#include "drivers/gyroscope.hpp"
#include "drivers/magnetometer.hpp"
#include "communication/state.hpp"

extern "C"
{
#include "util/quaternions.h"
}

#define GYRO_LPF 0.1f                       ///< The gyroscope linear pass filter gain
#define ACC_LPF 0.05f                       ///< The accelerometer linear pass filter gain
#define MAG_LPF 0.1f                        ///< The magnetometer linear pass filter gain


/**
 * \brief Sensor configuration
 */
typedef struct
{
    std::array<float, 3>    bias;         ///< The biais of the sensor
    std::array<float, 3>    scale_factor; ///< The scale factors of the sensor
    std::array<float, 3>    sign;         ///< The orientation of the sensor (+1 or -1)
    std::array<uint8_t, 3>  axis;         ///< The axis number (X,Y,Z) referring to the sensor datasheet
    std::array<float, 3>    max_values;   ///< Used only during calibration: max scaled value
    std::array<float, 3>    min_values;   ///< Used only during calibration: min scaled value
    std::array<float, 3>    mean_values;  ///< Used only during calibration: mean scaled value
} imu_sensor_config_t;


/**
 * \brief The configuration IMU structure
 */
typedef struct
{
    imu_sensor_config_t accelerometer;  ///< The gyroscope configuration structure
    imu_sensor_config_t gyroscope;      ///< The accelerometer configuration structure
    imu_sensor_config_t magnetometer;   ///< The compass configuration structure
    std::array<float, 3> magnetic_north;///< X, Y and Z components of magnetic north vector (includes magnetic inclination)
    float lpf_acc;                      ///< Low pass filter gain for accelerometer
    float lpf_gyro;                     ///< Low pass filter gain for accelerometer
    float lpf_mag;                      ///< Low pass filter gain for accelerometer
    float lpf_mean;                     ///< Low pass filter gain for the mean values
    float startup_calib_gyro_threshold; ///< Threshold on gyroscope value used to decide wheter the autopilot is held stable
    float startup_calib_duration_s;     ///< Duration in seconds of the automatic startup calibration of gyroscopes
} imu_conf_t;


/**
 * @brief   Default configuration
 *
 * @return  Config structure
 */
static inline imu_conf_t imu_default_config(void);


/**
 * \brief   Inertial measurement unit (IMU)
 *
 * \details This module gathers new data from inertial sensors and takes care of
 *          rotating, removing bias, and scaling raw sensor values (in this order)
 *
 *          If this module is used, then it is not needed to call each sensor's
 *          update function.
 */
class Imu
{
public:
    /**
     * \brief Constructor
     */
    Imu(Accelerometer& accelerometer,
        Gyroscope& gyroscope,
        Magnetometer& magnetometer,
        imu_conf_t config = imu_default_config());


    /**
     * \brief   Main update
     *
     * \return  Success
     */
    bool update(void);


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Value
     */
    const float& last_update_us(void) const;


    /**
     * \brief   Get time between two last updates in seconds
     *
     * \return  Value
     */
    const float& dt_s(void) const;


    /**
     * \brief   Get X, Y and Z components of acceleration in g
     *
     * \return  Value
     */
    const std::array<float, 3>& acc(void) const;


    /**
     * \brief   Get X, Y and Z components of angular velocity in rad/s
     *
     * \return  Value
     */
    const std::array<float, 3>& gyro(void) const;


    /**
     * \brief   Get X, Y and Z components of magnetic field (normalized)
     *
     * \return  Value
     */
    const std::array<float, 3>& mag(void) const;

    /**
     * \brief   Get X, Y and Z components of magnetic north (in NED frame)
     *
     * \return  Value
     */
    const std::array<float, 3>& magnetic_north(void) const;


    /**
     * \brief   Get the readiness of the IMU
     *
     * \return  True if the IMU is ready, false otherwise
     */
    bool is_ready(void) const;

    /**
     * \brief   Temporary method to get pointer to configuration
     *
     * \detail  Used to add onboard parameters for the biases and scales
     *          TODO: make it possible to set/get parameters via methods calls
     *          instead of via pointers and remove this method
     */
    imu_conf_t* get_config(void);

    // -------------------------------------------------------------------------
    //
    // TODO implement calibration
    //
    // -------------------------------------------------------------------------

    /**
     * \brief   Start accelerometer bias calibration
     *
     * \detail  Should not be used in flight
     *          Keep the platform perfectly level during the calibration phase
     *
     *          WARNING:
     *          Compatible with gyroscope bias calibration
     *          Incompatible with magnetometer bias calibration
     *
     * \return  Success if not already started and no incompatible calibration ongoing
     */
    bool start_accelerometer_bias_calibration(void);


    /**
     * \brief   Start gyroscope bias calibration
     *
     * \detail  Should not be used in flight
     *          Keep the platform perfectly level during the calibration phase
     *
     *          WARNING:
     *          Compatible with accelerometer bias calibration
     *          Incompatible with magnetometer bias calibration
     *
     * \return  Success if not already started and no incompatible calibration ongoing
     */
    bool start_gyroscope_bias_calibration(void);


    /**
     * \brief   Start magnetometer bias calibration
     *
     * \detail  Should not be used in flight
     *          Rotate the platform in all possible orientation during calibration
     *          The goal is to capture the maximum and minimum values for each axis,
     *          (ie. have X, Y and Z axis of the UAV). Consecutively, each axis
     *          should be perfectly aligned, then perfectly opposed to the
     *          magnetic field. Note that the magnetic field is not horizontal !
     *
     *          WARNING:
     *          Incompatible with accelerometer and gyroscope bias calibration
     *
     * \return  Success if not already started and no incompatible calibration ongoing
     */
    bool start_magnetometer_bias_calibration(void);


    /**
     * \brief   Start calibration of magnetic field inclination
     *
     * \detail  Should not be used in flight
     *          Keep the platform perfectly stable during the calibration phase. It does not have to
     *          be perfectly leveled, but should not undergo parasitic accelerations.
     *
     *          WARNING:
     *          Incompatible with magnetometer bias calibration
     *
     * \return  Success if not already started and no incompatible calibration ongoing
     */
    bool start_magnetic_north_calibration(void);


    /**
     * \brief   Stop accelerometer bias calibration
     *
     * \detail  Should not be used in flight
     *
     * \return  Success if not already stopped
     */
    bool stop_accelerometer_bias_calibration(void);


    /**
     * \brief   Stop gyroscope bias calibration
     *
     * \detail  Should not be used in flight
     *
     * \return  Success if not already stopped
     */
    bool stop_gyroscope_bias_calibration(void);


    /**
     * \brief   Stop magnetometer bias calibration
     *
     * \detail  Should not be used in flight
     *          This function updates the magnetometer bias with the
     *
     * \return  Success if not already stopped
     */
    bool stop_magnetometer_bias_calibration(void);


    /**
     * \brief   Stop calibration of magnetic inclination
     *
     * \detail  Should not be used in flight
     *
     * \return  Success if not already stopped
     */
    bool stop_magnetic_north_calibration(void);


private:
    /**
     * \brief   Tests if a calibration is ongoing
     *
     * \return  boolean
     */
    bool is_calibration_ongoing(void) const;

    /**
     * \brief   Startup calibration
     *
     * \detail  Should not be used in flight
     *          If the imu is not ready, this function waits for the gyroscopes
     *          values to be stable, then perform gyro bias calibration
     */
    void do_startup_calibration(void);


    /**
     * \brief   Performs ongoing calibrations
     *
     * \detail  Should not be used in flight
     */
    void do_calibration(void);


    Accelerometer&  accelerometer_;     ///< Reference to accelerometer sensor
    Gyroscope&      gyroscope_;         ///< Reference to gyroscope sensor
    Magnetometer&   magnetometer_;      ///< Reference to magnetometer sensor

    imu_conf_t      config_;            ///< Configuration

    std::array<float, 3> oriented_acc_; ///< Oriented acceleration
    std::array<float, 3> oriented_gyro_;///< Oriented angular velocity
    std::array<float, 3> oriented_mag_; ///< Oriented magnetic field

    std::array<float, 3> scaled_acc_;   ///< Scaled acceleration
    std::array<float, 3> scaled_gyro_;  ///< Scaled angular velocity
    std::array<float, 3> scaled_mag_;   ///< Scaled magnetic field
    float magnetic_inclination_;        ///< Angle between horizontal plane and magnetic north (magnetic dip)
    float magnetic_norm_;               ///< Norm of magnetic north

    bool do_startup_calibration_;               ///< Flag indicating if calibration should be done
    bool do_accelerometer_bias_calibration_;    ///< Flag indicating if calibration should be done
    bool do_gyroscope_bias_calibration_;        ///< Flag indicating if calibration should be done
    bool do_magnetometer_bias_calibration_;     ///< Flag indicating if calibration should be done
    bool do_magnetic_north_calibration_;        ///< Flag indicating if calibration should be done
    bool is_ready_;                             ///< Flag indicating the readiness of the IMU

    float dt_s_;                        ///< Time interval between two updates (in microseconds)
    float last_update_us_;              ///< Last update time in microseconds
    float timestamp_gyro_stable;        ///< The time from which the gyroscope is not varying too much
};



/**
 * @brief   Default configuration
 *
 * @return  Config structure
 */
static inline imu_conf_t imu_default_config()
{
    imu_conf_t conf = {};

    // Accelerometer
    // Bias
    conf.accelerometer.bias[0] = 0.0f;      ///< Positive or negative
    conf.accelerometer.bias[1] = 0.0f;
    conf.accelerometer.bias[2] = 0.0f;

    // Scale
    conf.accelerometer.scale_factor[0] = 1.0f;      ///< Should be >0
    conf.accelerometer.scale_factor[1] = 1.0f;
    conf.accelerometer.scale_factor[2] = 1.0f;

    // Axis and sign
    conf.accelerometer.sign[0] = +1.0f; ///< +1 or -1
    conf.accelerometer.sign[1] = +1.0f;
    conf.accelerometer.sign[2] = +1.0f;
    conf.accelerometer.axis[0] = 0;     ///< Should be 0, 1, or 2
    conf.accelerometer.axis[1] = 1;
    conf.accelerometer.axis[2] = 2;

    // Min and max values
    conf.accelerometer.max_values[0] = -10000.0f;
    conf.accelerometer.max_values[1] = -10000.0f;
    conf.accelerometer.max_values[2] = -10000.0f;
    conf.accelerometer.min_values[0] = 10000.0f;
    conf.accelerometer.min_values[1] = 10000.0f;
    conf.accelerometer.min_values[2] = 10000.0f;
    conf.accelerometer.mean_values[0] = 0.0f;
    conf.accelerometer.mean_values[1] = 0.0f;
    conf.accelerometer.mean_values[2] = 0.0f;

    // Gyroscope
    // Bias
    conf.gyroscope.bias[0] = 0.0f;      ///< Positive or negative
    conf.gyroscope.bias[1] = 0.0f;
    conf.gyroscope.bias[2] = 0.0f;

    // Scale
    conf.gyroscope.scale_factor[0] = 1.0f;      ///< Should be >0
    conf.gyroscope.scale_factor[1] = 1.0f;
    conf.gyroscope.scale_factor[2] = 1.0f;

    // Axis and sign
    conf.gyroscope.sign[0] = +1.0f; ///< +1 or -1
    conf.gyroscope.sign[1] = +1.0f;
    conf.gyroscope.sign[2] = +1.0f;
    conf.gyroscope.axis[0] = 0;     ///< Should be 0, 1, or 2
    conf.gyroscope.axis[1] = 1;
    conf.gyroscope.axis[2] = 2;

    // Min and max values
    conf.gyroscope.max_values[0] = -10000.0f;
    conf.gyroscope.max_values[1] = -10000.0f;
    conf.gyroscope.max_values[2] = -10000.0f;
    conf.gyroscope.min_values[0] = 10000.0f;
    conf.gyroscope.min_values[1] = 10000.0f;
    conf.gyroscope.min_values[2] = 10000.0f;
    conf.gyroscope.mean_values[0] = 0.0f;
    conf.gyroscope.mean_values[1] = 0.0f;
    conf.gyroscope.mean_values[2] = 0.0f;

    // Magnetometer
    // Bias
    conf.magnetometer.bias[0] = 0.0f;       ///< Positive or negative
    conf.magnetometer.bias[1] = 0.0f;
    conf.magnetometer.bias[2] = 0.0f;

    // Scale
    conf.magnetometer.scale_factor[0] = 1.0f;       ///< Should be >0
    conf.magnetometer.scale_factor[1] = 1.0f;
    conf.magnetometer.scale_factor[2] = 1.0f;

    // Axis and sign
    conf.magnetometer.sign[0] = +1.0f;  ///< +1 or -1
    conf.magnetometer.sign[1] = +1.0f;
    conf.magnetometer.sign[2] = +1.0f;
    conf.magnetometer.axis[0] = 0;      ///< Should be 0, 1, or 2
    conf.magnetometer.axis[1] = 1;
    conf.magnetometer.axis[2] = 2;

    // Min and max values
    conf.magnetometer.max_values[0] = -10000.0f;
    conf.magnetometer.max_values[1] = -10000.0f;
    conf.magnetometer.max_values[2] = -10000.0f;
    conf.magnetometer.min_values[0] = 10000.0f;
    conf.magnetometer.min_values[1] = 10000.0f;
    conf.magnetometer.min_values[2] = 10000.0f;
    conf.magnetometer.mean_values[0] = 0.0f;
    conf.magnetometer.mean_values[1] = 0.0f;
    conf.magnetometer.mean_values[2] = 0.0f;

    // Magnetic north (default value in Lausanne: 62° inclination)
    conf.magnetic_north[0] = 0.46947156f;   // cos(62°)
    conf.magnetic_north[1] = 0.0f;
    conf.magnetic_north[2] = 0.88294759f;   // sin(62°)

    // Low pass filter
    conf.lpf_acc    = 0.1f;
    conf.lpf_gyro   = 0.05f;
    conf.lpf_mag    = 0.1f;
    conf.lpf_mean   = 0.01f;

    // startup calibration length
    conf.startup_calib_gyro_threshold = 0.5f;
    conf.startup_calib_duration_s     = 10.0f;

    return conf;
}

#endif /* IMU_HPP_ */
