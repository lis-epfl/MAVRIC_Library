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
 * \file toggle_logging.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Toggle the logging of data
 *
 ******************************************************************************/

#ifndef TOGGLE_LOGGING_HPP_
#define TOGGLE_LOGGING_HPP_

#include "communication/state.hpp"

extern "C"
{
#include <stdbool.h>
#include <stdint.h>
}

/**
 * \brief   Configuration for the module data logging
 */
typedef struct
{
    uint32_t max_data_logging_count;            ///< Maximum number of parameters
    uint16_t max_logs;                          ///< The max number of logged files with the same name on the SD card
    bool debug;                                 ///< Indicates if debug messages should be printed for each param change
    uint32_t log_data;                          ///< The initial state of writing a file
} toggle_logging_conf_t;

/**
 * \brief   The fat_fs mounting structure
 */
typedef struct
{
    toggle_logging_conf_t toggle_logging_conf;  ///< The data logging configuration structure

    uint32_t log_data;                          ///< A flag to stop/start writing to file

    const State* state;                         ///< The pointer to the state structure
} toggle_logging_t;

/**
 * \brief   Initialise the toggle logging system file
 *
 * \param   toggle_logging          The pointer to the toggle telemetry structure
 * \param   toggle_logging_conf     The pointer to the configuration structure
 * \param   state                   The pointer to the state structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool toggle_logging_init(toggle_logging_t* toggle_logging, toggle_logging_conf_t toggle_logging_conf, const State* state);

#endif /* fat_fs_mounting_TELEMETRY_HPP_ */
