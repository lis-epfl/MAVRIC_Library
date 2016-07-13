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
 * \file endian.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Implementation of endianness-dependant functions
 *
 ******************************************************************************/


#ifndef MAVRIC_ENDIAN_H_
#define MAVRIC_ENDIAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __MAVRIC_ENDIAN_BIG__
#ifndef __MAVRIC_ENDIAN_LITTLE__
#warning Unknown Endian, using default __MAVRIC_ENDIAN_LITTLE__
#define __MAVRIC_ENDIAN_LITTLE__
#endif
#endif

#include <stdint.h>

/**
 * @brief   Arrange a 16 bits number in little endian
 *
 * @param   data    Input in native format
 * @return          Output in little endian
 */
static inline uint16_t endian_to_little16(uint16_t data);


/**
 * @brief   Read a 16 bits number stored in little endian
 *
 * @param   data    Input in native format
 * @return          Output in little endian
 */
static inline uint16_t endian_from_little16(uint16_t data);


/**
 * @brief   Arrange a 16 bits number in big endian
 *
 * @param   data    Input in native format
 * @return          Output in big endian
 */
static inline uint16_t endian_to_big16(uint16_t data);


/**
 * @brief   Read a 16 bits number stored in big endian
 *
 * @param   data    Input in native format
 * @return          Output in big endian
 */
static inline uint16_t endian_from_big16(uint16_t data);


/**
 * @brief   Arrange a 32 bits number in little endian
 *
 * @param   data    Input in native format
 * @return          Output in little endian
 */
static inline uint32_t endian_to_little32(uint32_t data);


/**
 * @brief   Read a 32 bits number stored in little endian
 *
 * @param   data    Input in native format
 * @return          Output in little endian
 */
static inline uint32_t endian_from_little32(uint32_t data);


/**
 * @brief   Arrange a 32 bits number in big endian
 *
 * @param   data    Input in native format
 * @return          Output in big endian
 */
static inline uint32_t endian_to_big32(uint32_t data);


/**
 * @brief   Read a 32 bits number stored in big endian
 *
 * @param   data    Input in native format
 * @return          Output in big endian
 */
static inline uint32_t endian_from_big32(uint32_t data);


/**
 * @brief   Converts a 16 bits number from native to network endian
 *
 * @param   data    Input in native format
 * @return          Output in network endian
 */
static inline uint16_t endian_htons(uint16_t data);


/**
 * @brief   Converts a 16 bits number from network to native endian
 *
 * @param   data    Input in network format
 * @return          Output in native endian
 */
static inline uint16_t endian_ntohs(uint16_t data);


/**
 * @brief   Converts a 32 bits number from native to network endian
 *
 * @param   data    Input in native format
 * @return          Output in network endian
 */
static inline uint32_t endian_htonl(uint32_t data);


/**
 * @brief   Converts a 32 bits number from network to native endian
 *
 * @param   data    Input in network format
 * @return          Output in native endian
 */
static inline uint32_t endian_ntohl(uint32_t data);


/**
 * @brief   Reverses endianness in a 16 bits number
 *
 * @param   data    Input
 * @return          Output
 */
static inline uint16_t endian_rev16(uint16_t data)
{
    return ((data & 0x00FF) << 8) |
           ((data & 0xFF00) >> 8);
};


/**
 * @brief   Reverses endianness in a 16 bits signed number
 *
 * @param   data    Input
 * @return          Output
 */
static inline int16_t endian_rev16s(int16_t data)
{
    return ((data >> 8) & 0x00ff) | ((data & 0x00ff) << 8);
};


/**
 * @brief   Reverses endianness in a 32 bits number
 *
 * @param   data    Input
 * @return          Output
 */
static inline uint32_t endian_rev32(uint32_t data)
{
    return ((data & 0x000000FF) << 24) |
           ((data & 0x0000FF00) << 8)  |
           ((data & 0x00FF0000) >> 8)  |
           ((data & 0xFF000000) >> 24);
};



#ifdef __MAVRIC_ENDIAN_BIG__

static inline uint16_t endian_to_little16(uint16_t data)
{
    return endian_rev16(data);
}

static inline uint16_t endian_from_little16(uint16_t data)
{
    return endian_rev16(data);
}

static inline uint16_t endian_to_big16(uint16_t data)
{
    return data;
}

static inline uint16_t endian_from_big16(uint16_t data)
{
    return data;
}

static inline uint32_t endian_to_little32(uint32_t data)
{
    return endian_rev32(data);
}

static inline uint32_t endian_from_little32(uint32_t data)
{
    return endian_rev32(data);
}

static inline uint32_t endian_to_big32(uint32_t data)
{
    return data;
}

static inline uint32_t endian_from_big32(uint32_t data)
{
    return data;
}

static inline uint16_t endian_htons(uint16_t data)
{
    return data;
}

static inline uint16_t endian_ntohs(uint16_t data)
{
    return data;
}

static inline uint32_t endian_htonl(uint32_t data)
{
    return data;
}

static inline uint32_t endian_ntohl(uint32_t data)
{
    return data;
}

#endif


#ifdef __MAVRIC_ENDIAN_LITTLE__

static inline uint16_t endian_to_little16(uint16_t data)
{
    return data;
}

static inline uint16_t endian_from_little16(uint16_t data)
{
    return data;
}

static inline uint16_t endian_to_big16(uint16_t data)
{
    return endian_rev16(data);
}

static inline uint16_t endian_from_big16(uint16_t data)
{
    return endian_rev16(data);
}

static inline uint32_t endian_to_little32(uint32_t data)
{
    return data;
}

static inline uint32_t endian_from_little32(uint32_t data)
{
    return data;
}

static inline uint32_t endian_to_big32(uint32_t data)
{
    return endian_rev32(data);
}

static inline uint32_t endian_from_big32(uint32_t data)
{
    return endian_rev32(data);
}

static inline uint16_t endian_htons(uint16_t data)
{
    return endian_rev16(data);
}

static inline uint16_t endian_ntohs(uint16_t data)
{
    return endian_rev16(data);
}

static inline uint32_t endian_htonl(uint32_t data)
{
    return endian_rev32(data);
}

static inline uint32_t endian_ntohl(uint32_t data)
{
    return endian_rev32(data);
}

#endif

#ifdef __cplusplus
}
#endif

#endif /* MAVRIC_ENDIAN_H_ */
