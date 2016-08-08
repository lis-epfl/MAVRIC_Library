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

#define WHO_ARE_YOU_COMMAND 0x75
#define MPU_9250_WHOAMI_ID  0x71

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mpu_9250::Mpu_9250(Spi& spi):
    spi_(spi),
    gyro_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    acc_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    temperature_(0.0f),
    last_update_us_(0.0f)
{}

bool Mpu_9250::init(void)
{
    bool success = true;

    //who am I?
    //uint8_t data;
    //success &= spi_.write(WHO_ARE_YOU_COMMAND);
    //success &= spi_.read(&data);

    return success;
}


bool Mpu_9250::update_acc(void)
{
    bool success = true;

    return success;
}


bool Mpu_9250::update_gyr(void)
{
    bool success = true;

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


const float& Mpu_9250::temperature(void) const
{
    return temperature_;
}

 int32_t MPU_9250_Test(Spi_stm32& spi)
{

	uint8_t reg = 0x75;
	uint8_t id = 0;
	spi.write(&reg, 1);
	spi.read(&id, 1);

	if (id != MPU_9250_WHOAMI_ID)
		return 1;

	// id = PIOS_MPU9250_Mag_ReadReg(AK8963_WHOAMI_REG);
	// if (id != AK8963_WHOAMI_ID)
	// 	return -2;

	return 0;
}