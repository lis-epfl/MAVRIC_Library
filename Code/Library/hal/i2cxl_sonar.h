// Copyright (C) 2014  Julien Lecoeur

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#ifndef I2CXL_SONAR_H_
#define I2CXL_SONAR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"

typedef struct {
	uint8_t i2c_address;
	uint16_t distance_cm;
	float distance_m;
} i2cxl_sonar_t;


/**
 * @brief I2CXL sonar initialisation
 * @details Initialises the data struct and the i2c bus
 * 
 * @param i2c_sonar Data struct 
 */
void i2cxl_sonar_init(i2cxl_sonar_t* i2c_sonar);


/**
 * @brief Reads last value from sensor and start new recording
 * @details This function should be called at a frequency lower
 * than 10Hz
 * 
 * @param i2c_sonar Data struct
 */
void i2cxl_sonar_update(i2cxl_sonar_t* i2c_sonar);

#ifdef __cplusplus
}
#endif

#endif /* I2CXL_SONAR_H */