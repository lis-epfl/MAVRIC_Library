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
 * \file buffer.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Buffer
 *
 ******************************************************************************/


#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#include <stdint.h>
#include <stdbool.h>


/**
 * \brief Circular buffer
 *
 * \tparam S    Size of the buffer (256 by default)
 * \tparam T    Type of buffered data (uint8_t by default)
 */
template<uint32_t S = 256, typename T = uint8_t>
class Buffer_tpl
{
public:
    /**
     * \brief Constructor
     */
    Buffer_tpl(void);


    /**
     * \brief           Stores data in the buffer even of it overwrites existing data
     *
     * \param   data    data to write
     *
     * \return  Success
     */
    bool put_lossy(const T& data);


    /**
     * \brief           Stores data in the buffer, until the buffer is full
     *
     * \param   data    data to write
     *
     * \return  Success
     */
    bool put(const T& data);


    /**
     * \brief           Get the oldest element in the buffer
     *
     * \param   data    Data read
     *
     * \return          Success
     */
    bool get(T& data);


    /**
     * \brief           Clear the buffer
     *
     * \details         This function erases the buffer, note that it does not
     *                  erase all the bytes one by one so the function call is fast
     *
     * \param buffer    Pointer to buffer
     */
    void clear(void);


    /**
     * \brief           Returns the number of available bytes in the buffer
     *
     * \return          Number of available bytes
     */
    uint32_t readable(void) const;


    /**
     * \brief           Returns the number of free bytes in the buffer
     *
     * \return          Number of free bytes
     */
    uint32_t writeable(void) const;


    /**
     * \brief           Tests whether the buffer is full
     *
     * \return          Boolean, 1 if full, 0 if not
     */
    bool full(void) const;


    /**
     * \brief           Tests whether the buffer is empty
     *
     * \return          Boolean, 1 if empty, 0 if not
     */
    bool empty(void) const;


private:
    T           buffer_[S + 1]; ///<    Array of bytes containing the data
    uint32_t    head_;          ///<    Head of the buffer (newest byte)
    uint32_t    tail_;          ///<    Tail of the buffer (oldest byte)
};


/**
 * \brief   Default buffer
 *
 * \detail  This is to make the code more readable in most cases when templates
 *          are not needed. The default buffer has size 256 and type uint8_t.
 */
typedef Buffer_tpl<256, uint8_t> Buffer;


#include "buffer.hxx"

#endif /* BUFFER_HPP_ */