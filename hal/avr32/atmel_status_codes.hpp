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
 * \file atmel_status_codes.h
 *
 * \author MAV'RIC Team
 *
 * \brief Translation from Atmel status_code to debug messages and booleans
 *
 ******************************************************************************/

#ifndef ATMEL_STATUS_CODES_H_
#define ATMEL_STATUS_CODES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "libs/asf/avr32/utils/status_codes.h"
#include "util/print_util.h"


/**
 * @brief   Converts status codes from Atmel framework to boolean
 *
 * @param   status  Status code provided by Atmel functions
 * @param   debug   Prints messages if true
 *
 * @return  true    STATUS_OK
 * @return  false   Not STATUS_OK
 */
bool status_code_to_bool(status_code_t status, bool debug = false)
{
    bool res = false;

    if (debug == false)
    {
        if (status == STATUS_OK)
        {
            res = true;
        }
        else
        {
            res = false;
        }
    }
    else
    {
        switch (status)
        {
            case STATUS_OK:
                res = true;
                break;
            case ERR_IO_ERROR:
                res = false;
                print_util_dbg_print("[ERR] IO");
                break;
                break;
            case ERR_FLUSHED:
                res = false;
                print_util_dbg_print("[ERR] Flushed");
                break;
            case ERR_TIMEOUT:
                res = false;
                print_util_dbg_print("[ERR] Timeout");
                break;
            case ERR_BAD_DATA:
                res = false;
                print_util_dbg_print("[ERR] Bad data");
                break;
            case ERR_PROTOCOL:
                res = false;
                print_util_dbg_print("[ERR] Protocol");
                break;
            case ERR_UNSUPPORTED_DEV:
                res = false;
                print_util_dbg_print("[ERR] Unsupported dev");
                break;
            case ERR_NO_MEMORY:
                res = false;
                print_util_dbg_print("[ERR] No Memory");
                break;
            case ERR_INVALID_ARG:
                res = false;
                print_util_dbg_print("[ERR] Invalid arg");
                break;
            case ERR_BAD_ADDRESS:
                res = false;
                print_util_dbg_print("[ERR] Bad address");
                break;
            case ERR_BUSY:
                res = false;
                print_util_dbg_print("[ERR] Busy");
                break;
            case ERR_BAD_FORMAT:
                res = false;
                print_util_dbg_print("[ERR] Bad format");
                break;
            default:
                res = false;
                break;

        }
    }

    return res;
}


#ifdef __cplusplus
}
#endif

#endif /* ATMEL_STATUS_CODES_H_ */