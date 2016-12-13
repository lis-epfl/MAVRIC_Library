/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file px4flow_serial.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for optic flow sensors
 *
 ******************************************************************************/

#ifndef PX4FLOW_SERIAL_HPP_
#define PX4FLOW_SERIAL_HPP_

#include "drivers/px4flow.hpp"
#include "communication/mavlink_stream.hpp"
#include <cstdint>
#include "hal/common/serial.hpp"


/**
 * \brief   Driver for PX4Flow
 */
class  PX4Flow_serial: public PX4Flow
{
public:
    /**
     * \brief Configuration
     */
    struct conf_t
    {
        orientation_t orientation;
    };

    /**
     * \brief Constructor
     *
     * \param    uart    Pointer to serial peripheral
     */
    PX4Flow_serial(Serial& uart, conf_t config = default_config());

    /**
    * \brief    Update function
    * \return   Success
    */
    bool update(void);

    /**
     * \brief   Default configuration
     *
     * \return  Configuration structure
     */
    static inline conf_t default_config(void);

private:
    Serial&             uart_;               ///< Serial device
    Mavlink_stream      mavlink_stream_;     ///< Mavlink interface using streams

    conf_t              config_;            ///< Configuration
};


/**
 * \brief   Default configuration
 *
 * \return  Configuration structure
 */
PX4Flow_serial::conf_t PX4Flow_serial::default_config(void)
{
    conf_t conf = {};

    conf.orientation = ORIENT_0_DEG;

    return conf;
}

#endif /* PX4FLOW_SERIAL_HPP_ */
