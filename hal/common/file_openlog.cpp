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
 * \file    file_openlog.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Files that just writes to  an openlog through serial port
 *
 ******************************************************************************/

#include "hal/common/file_openlog.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"

File_openlog::File_openlog(Serial& serial, bool debug):
    serial_(serial),
    is_open_(false),
    debug_(debug)
{
    // Init file name
    file_name_[0] = 'f';
    file_name_[1] = 'i';
    file_name_[2] = 'l';
    file_name_[3] = 'e';
    file_name_[4] = '\0';
}


bool File_openlog::open(const char* path)
{
    is_open_ = true;
    uint8_t filename_length = 0;
    for (uint32_t i = 0; i < MAX_FILENAME_LENGTH; i++)
    {
        file_name_[i] = path[i];
        if (path[i] == '\0')
        {
            break;
        }
        else
        {
            filename_length += 1;
        }
    }

    if (debug_)
    {
        print_util_dbg_print("Opening file:");
        print_util_dbg_print(file_name_);
        print_util_dbg_print("\r\n");
    }

    // // Enter command mode
    // const uint8_t command_mode[3] = {26, 26, 26};
    // serial_.write(command_mode, 3);
    //
    // time_keeper_delay_ms(20);
    //
    // // Open new file
    // const char* command_new = "append ";
    // serial_.write((uint8_t*)command_new, 7);
    // serial_.write((uint8_t*)file_name_, filename_length);

    // Write file name
    const char* separator = "\n\nNEW FILE ";
    serial_.write((uint8_t*)separator, 11);
    serial_.write((uint8_t*)path, filename_length);
    serial_.write((uint8_t*)separator, 1);

    return true;
}


bool File_openlog::is_open()
{
    return is_open_;
}


int8_t File_openlog::exists(const char* path)
{
    // return 0;

    // // Print path
    // print_util_dbg_print("Checking file:");
    // print_util_dbg_print(path);
    // print_util_dbg_print("\r\n");
    //
    // // Enter command mode
    // const uint8_t command_mode[3] = {26, 26, 26};
    // serial_.write(command_mode, 3);
    //
    // time_keeper_delay_ms(20);
    //
    // // Flush incoming bytes
    // uint32_t to_read = 0;
    // uint8_t buf[255];
    // while (serial_.readable() > 0)
    // {
    //     serial_.read(buf, 1);
    // }
    //
    // // Check size of file
    // uint8_t path_length = 0;
    // for (uint32_t i = 0; i < MAX_FILENAME_LENGTH; i++)
    // {
    //     if (path[i] == '\0')
    //     {
    //         break;
    //     }
    //     else
    //     {
    //         path_length += 1;
    //     }
    // }
    // const char* command_size = "size ";
    // serial_.write((uint8_t*)command_size, 5);
    // // const char* command_cat = "cat ";
    // // serial_.write((uint8_t*)command_cat, 4);
    // serial_.write((uint8_t*)path, path_length);
    // // serial_.flush();
    //
    // time_keeper_delay_ms(200);
    //
    // uint32_t size = 0;
    // to_read = serial_.readable();
    //
    // print_util_dbg_print("Openlog to read ");
    // print_util_dbg_print_num(to_read, 10);
    // print_util_dbg_print("\r\n");
    //
    // if (to_read > 0)
    // {
    //     serial_.read(buf, to_read);
    //
    //     // Convert to number
    //     for (uint8_t i = 0; i < to_read; i++)
    //     {
    //         uint8_t digit = buf[i] - '0';
    //         print_util_dbg_print_num(digit, 10);
    //         if ((0 <= digit) && (digit <= 9))
    //         {
    //             size += digit * pow(10, to_read - i);
    //         }
    //         else
    //         {
    //             size = 0;
    //         }
    //     }
    //
    //     print_util_dbg_print("File size ");
    //     print_util_dbg_print_num(size, 10);
    //     print_util_dbg_print("\r\n");
    //
    //
    //     if (size > 0)
    //     {
    //         return 1;
    //     }
    //     else
    //     {
    //         return 0;
    //     }
    // }
    // else
    // {
    //     // Error
    //     return 0;
    // }

    // Check if names are same
    bool same_names = true;
    for (uint32_t i = 0; i < MAX_FILENAME_LENGTH; i++)
    {
        if (path[i] != file_name_[i])
        {
            // names are different
            same_names = false;
            break;
        }
        else if ((path[i] == '\0') || (file_name_[i] == '\0'))
        {
            if ((path[i] == '\0') && (file_name_[i] == '\0'))
            {
                // Reached end of both names -> equal
                same_names = true;
                break;
            }
            else
            {
                // One name is longer -> different
                same_names = false;
                break;
            }
        }
    }

    if (debug_)
    {
        if (same_names)
        {
            print_util_dbg_print("File: ");
            print_util_dbg_print(file_name_);
            print_util_dbg_print(" already exists.\r\n");
        }
        else
        {
            print_util_dbg_print("File: ");
            print_util_dbg_print(file_name_);
            print_util_dbg_print(" does not exist.\r\n");
        }
    }

    if (same_names)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


bool File_openlog::close()
{
    is_open_ = false;
    return true;
}


bool File_openlog::read(uint8_t* data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        data[i] = 0;
    }

    return true;
}


bool File_openlog::write(const uint8_t* data, uint32_t size)
{
    return serial_.write(data, size);
    return true;
}


bool File_openlog::seek(int32_t offset, file_seekfrom_t origin)
{
    return true;
}


uint32_t File_openlog::offset()
{
    return 0;
}


uint32_t File_openlog::length()
{
    return 0;
}


bool File_openlog::flush()
{
    return true;
}
