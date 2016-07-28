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
 * \file    file_flash_avr32.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Class to read and write to flash memory on avr32
 *
 ******************************************************************************/

#ifndef FILE_FLASH_AVR32_HPP_
#define FILE_FLASH_AVR32_HPP_

#include "hal/common/file.hpp"


/**
 * \brief   File objects on linux platforms
 */
class File_flash_avr32: public File
{
private:
    uint32_t offset_;   ///< Current position in flash memory

public:
    /**
     * \brief   Constructor
     */
    File_flash_avr32(const char* path);


    /**
     * \brief   Open the file
     *
     * \return  true if the file is open, false otherwise
     */
    bool open(const char* path);


    /**
     * \brief   Indicates if the file is currently open
     *
     * \return  true if the file is open, false otherwise
     */
    bool is_open();

    /**
     * \brief   Indicates if the file exits
     *
     * \return  1 if the file exists, 0 if the file doesn't exist, -1 if the file system is not responding properly
     */
    int8_t exists(const char* path);

    /**
     * \brief   Close the file
     *
     * \return  success
     */
    bool close();


    /**
     * \brief   Read from the file.
     *
     * \param   data        Caller supplied buffer to write to.
     * \param   size        The number of bytes to attempt to read.
     *
     * \return  success
     */
    bool read(uint8_t* data, uint32_t size);


    /**
     * \brief   Write to the file.
     *
     * \param   data    The buffer to write.
     * \param   size    The number of bytes to write.
     *
     * \return  success
     */
    bool write(const uint8_t* data, uint32_t size);


    /**
     * \brief   Seek to a given offset within the file.
     *
     * \details     Valid locations to seek to are from zero to the file length.
     *              Seeking to the file length moves the pointer to one past the
     *              end of the file data so that subsequent file writes append
     *              to the existing file.
     *
     * \param   offset      The distance to move from the origin_ parameter.
     * \param   origin      One of the SeekFrom enumeration.
     *
     * \return  success
     */
    bool seek(int32_t offset, file_seekfrom_t origin);


    /**
     * \brief   Get current location in file
     *
     * \return  Offset in bytes
     */
    uint32_t offset();


    /**
     * \brief   Get the file length.
     *
     * \return  The file length up to the maximum supported 4Gb.
     */
    uint32_t length();

    /**
     * \brief   flush buffer to file
     *
     * \return  success (always true)
     */
    bool flush();
};

#endif /* FILE_FLASH_AVR32_HPP_ */