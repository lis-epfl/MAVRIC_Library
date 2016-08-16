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
 * \file data_logging.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Performs the data logging on a file
 *
 ******************************************************************************/


#include "communication/data_logging.hpp"

#include <cmath>
#include <string>

#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.hpp"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Data_logging::add_header_name(void)
{
    bool init = true;

    uint16_t i;

    init &= console_.write("time,");

    for (i = 0; i < data_logging_count_; i++)
    {
        data_logging_entry_t* param = &data_log_[i];

        init &= console_.write(reinterpret_cast<uint8_t*>(param->param_name), strlen(param->param_name));
        write_separator(i);

        if (!init)
        {
            if (debug_)
            {
                print_util_dbg_print("Error appending header!\r\n");
            }
        }
    }

    file_init_ = init;
}


void Data_logging::write_separator(uint16_t param_num)
{
    bool success = true;

    if (param_num == (data_logging_count_ - 1))
    {
        // Last variable -> end of line
        success &= console_.write("\n");
    }
    else
    {
        // Not last variable -> separator
        success &= console_.write(",");
    }

    if (!success)
    {
        if (debug_)
        {
            print_util_dbg_print("Error putting tab or new line character!\r\n");
        }
    }
}


void Data_logging::log_parameters(void)
{
    uint32_t i;
    bool success = true;

    // First parameter is always time
    uint32_t time_ms = time_keeper_get_ms();
    success &=  console_.write(time_ms);
    success &=  console_.write(",");

    for (i = 0; i < data_logging_count_; i++)
    {
        // Writing the value of the parameter to the file, separate values by tab character
        data_logging_entry_t* param = &data_log_[i];
        switch (param->data_type)
        {
            case MAV_PARAM_TYPE_UINT8:
                success &= console_.write(*((uint8_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_INT8:
                success &= console_.write(*((int8_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_UINT16:
                success &= console_.write(*((uint16_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_INT16:
                success &= console_.write(*((int16_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_UINT32:
                success &= console_.write(*((uint32_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_INT32:
                success &= console_.write(*((int32_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_UINT64:
                success &= console_.write(*((uint64_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_INT64:
                success &= console_.write(*((int64_t*)param->param));
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_REAL32:
                success &= console_.write(*(float*)param->param, param->precision);
                write_separator(i);
                break;

            case MAV_PARAM_TYPE_REAL64:
                success &= console_.write(*((double*)param->param), param->precision);
                write_separator(i);
                break;

            default:
                success &= false;
                write_separator(i);
                break;
        }

        if (!success)
        {
            if (debug_)
            {
                print_util_dbg_print("Error appending parameter! Error:");
            }
        }
    }
}


void Data_logging::seek(void)
{
    bool success = true;

    /* Seek to end of the file to append data */
    success &= console_.get_stream()->seek(0, FILE_SEEK_END);

    if (!success)
    {
        if (debug_)
        {
            print_util_dbg_print("lseek error:");
        }
        // Closing the file if we could not seek the end of the file
        console_.get_stream()->close();
    }
}


bool Data_logging::filename_append_extension(char* output, char* filename, uint32_t length)
{
    // Success flag
    bool is_success = true;

    // Declare counter for char location
    uint32_t i = 0;

    // Copy characters to output from filename until null character is reached
    while (filename[i] != '\0')
    {
        output[i] = filename[i];
        i++;

        // If i is one less than length
        if (i == (length - 1))
        {
            // Set last character of output to null character
            output[i] = '\0';

            // Return is_success as false
            is_success = false;
            return is_success;
        }
    }

    // If there is not enough room for .txt\0
    if ((i + 5) >= (length))
    {
        // Set last character of output to null, dont append
        // .txt
        output[i] = '\0';

        // Return is_success as false
        is_success = false;
        return is_success;
    }

    // Add ".csv"
    output[i] = '.';
    output[i + 1] = 'c';
    output[i + 2] = 's';
    output[i + 3] = 'v';

    // Add null character
    output[i + 4] = '\0';

    // Return is_success as true;
    return is_success;
}


bool Data_logging::filename_append_int(char* output, char* filename, uint32_t num, uint32_t length)
{
    // Success flag
    bool is_success = true;

    // Declare counter for char location
    uint32_t i = 0;

    // Copy characters to output from filename until null character is
    while (filename[i] != '\0')
    {
        output[i] = filename[i];
        i++;

        // If i is one less than length
        if (i == (length - 1))
        {
            // Set last character of output to null character
            output[i] = '\0';

            // Return is_success as false
            is_success = false;
            return is_success;
        }
    }

    // Add underscore
    output[i] = '_';

    // Count number of digits
    uint32_t num_digits = 0;
    uint32_t num_copy = num;
    do // Do while loop to have 0 written as 1 digit
    {
        num_digits++; // Add one to digits
        num_copy = num_copy / 10; // Remove digit from num_copy
    }
    while (num_copy != 0);

    // If the number of digits + i is greater than or equal to length - 1
    if ((num_digits + i) >= (length - 1))
    {
        // Not enough space is allocated
        // Set null character (overwrite _ since number wont be outputted)
        output[i] = '\0';

        // Return false
        is_success = false;
        return is_success;
    } // If not, then there is enough space and continue

    // Add num_digits to i
    i += num_digits;
    do // Remove digits right to left adding them to output
    {
        output[i] = (num % 10) + '0';
        num = num / 10;
        i--; // Subtrack as we are moving right to left
    }
    while (num != 0);   // Stop when rev_num has gone through all the digits

    // Add null character to i+num_digits+1
    output[i + num_digits + 1] = '\0';

    return is_success;
}


bool Data_logging::open_new_log_file(void)
{
    bool create_success = true;

    uint32_t i = 0;

    if (log_data_)
    {
        do
        {
            // Create flag for successfully written file names
            bool successful_filename = true;

            // Add iteration number to name_n_extension_ (does not yet have extension)
            successful_filename &= filename_append_int(name_n_extension_, file_name_, i, MAX_FILENAME_LENGTH);

            // Add extension (.txt) to name_n_extension_
            successful_filename &= filename_append_extension(name_n_extension_, name_n_extension_, MAX_FILENAME_LENGTH);

            // Check if there wasn't enough memory allocated to name_n_extension_
            if (!successful_filename)
            {
                print_util_dbg_print("Name error: The name is too long! It should be, with the extension, maximum ");
                print_util_dbg_print_num(MAX_FILENAME_LENGTH, 10);
                print_util_dbg_print(" and it is ");
                print_util_dbg_print_num(sizeof(name_n_extension_), 10);
                print_util_dbg_print("\r\n");

                create_success = false;
            }

            // If the filename was successfully created, try to open a file
            if (successful_filename)
            {
                int8_t exists = console_.get_stream()->exists(name_n_extension_);
                switch (exists)
                {
                    case -1:
                        sys_status_ = false;
                        create_success = false;
                        break;

                    case 0:
                        sys_status_ = true;
                        create_success = console_.get_stream()->open(name_n_extension_);
                        break;

                    case 1:
                        sys_status_ = true;
                        create_success = false;
                        break;
                }

            }

            if (debug_)
            {
                print_util_dbg_print("Open result:");
                print_util_dbg_print_num(create_success, 10);
                print_util_dbg_print("\r\n");
            }

            ++i;
        }
        while ((i < config_.max_logs) && (!create_success) && sys_status_);

        if (create_success)
        {
            seek();

            file_opened_ = true;

            if (debug_)
            {
                print_util_dbg_print("File ");
                print_util_dbg_print(name_n_extension_);
                print_util_dbg_print(" opened. \r\n");
            }
        } //end of if fr == FR_OK
    }//end of if (log_data_)

    return create_success;
}


bool Data_logging::checksum_control(void)
{
    bool new_values = false;

    double cksum_a_current = 0.0;
    double cksum_b_current = 0.0;

    float approx = 1.0f;

    uint32_t j = 0;

    for (uint32_t i = 0; i < data_logging_count_; ++i)
    {
        data_logging_entry_t* param = &data_log_[i];
        switch (param->data_type)
        {
            case MAV_PARAM_TYPE_UINT8:
                cksum_a_current += *((uint8_t*)param->param);
                break;

            case MAV_PARAM_TYPE_INT8:
                cksum_a_current += *((int8_t*)param->param);
                break;

            case MAV_PARAM_TYPE_UINT16:
                cksum_a_current += *((uint16_t*)param->param);
                break;

            case MAV_PARAM_TYPE_INT16:
                cksum_a_current += *((int16_t*)param->param);
                break;

            case MAV_PARAM_TYPE_UINT32:
                cksum_a_current += *((uint32_t*)param->param);
                break;

            case MAV_PARAM_TYPE_INT32:
                cksum_a_current += *((int32_t*)param->param);
                break;

            case MAV_PARAM_TYPE_UINT64:
                cksum_a_current += *((uint64_t*)param->param);
                break;

            case MAV_PARAM_TYPE_INT64:
                cksum_a_current += *((int64_t*)param->param);
                break;

            case MAV_PARAM_TYPE_REAL32:
                for (j = 0; j < param->precision; ++j)
                {
                    approx *= 10;
                }
                approx = round((*((float*)param->param)) * approx) / approx;
                cksum_a_current += approx;
                break;

            case MAV_PARAM_TYPE_REAL64:
                for (j = 0; j < param->precision; ++j)
                {
                    approx *= 10;
                }
                approx = round((*((double*)param->param)) * approx) / approx;
                cksum_a_current += approx;
                break;
            default:
                cksum_a_current = 0.0;
                cksum_b_current = 0.0;
                print_util_dbg_print("Data type not supported!\r\n");
                break;
        }
        cksum_b_current += cksum_a_current;
    }

    //  print_util_dbg_print("cksum: (");
    //  print_util_dbg_print_num(cksum_a_current*100,10);
    //  print_util_dbg_print("==");
    //  print_util_dbg_print_num(cksum_a_*100,10);
    //  print_util_dbg_print(")&&(");
    //  print_util_dbg_print_num(cksum_b_current*100,10);
    //  print_util_dbg_print("==");
    //  print_util_dbg_print_num(cksum_b_*100,10);
    //  print_util_dbg_print(")\r\n");


    if ((cksum_a_current == cksum_a_) && (cksum_b_current == cksum_b_))
    {
        new_values = false;
    }
    else
    {
        new_values = true;
        cksum_a_ = cksum_a_current;
        cksum_b_ = cksum_b_current;
    }

    return new_values;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Data_logging::Data_logging(File& file, State& state, data_logging_conf_t config):
    config_(config),
    console_(file),
    state_(state)
{
    data_log_ = (data_logging_entry_t*)malloc(sizeof(data_logging_entry_t[config_.max_data_logging_count]));

    //in case malloc failed
    if (data_log_ == NULL)
    {
        config_.max_data_logging_count = 0;
    }

    log_data_ = config_.log_data;

    data_logging_count_ = 0;
}

bool Data_logging::create_new_log_file(const char* file_name__, uint32_t sysid)
{
    bool init_success = true;

    file_init_ = false;
    file_opened_ = false;
    sys_status_ = true;

    sys_id_ = sysid;

    // Append sysid to filename
    filename_append_int(file_name_, (char*)file_name__, sysid, MAX_FILENAME_LENGTH);

    init_success &= open_new_log_file();

    logging_time_ = time_keeper_get_ms();

    cksum_a_ = 0.0;
    cksum_b_ = 0.0;

    return init_success;
}


bool Data_logging::update(void)
{
    uint32_t time_ms = 0;
    if (log_data_ == 1)
    {
        if (file_opened_)
        {
            if (!file_init_)
            {
                add_header_name();
            }

            if (!state_.is_armed())
            {
                time_ms = time_keeper_get_ms();
                if ((time_ms - logging_time_) > 5000)
                {
                    console_.get_stream()->flush();
                    logging_time_ = time_ms;
                }
            }

            if (config_.continuous_write)
            {
                log_parameters();
            }
            else
            {
                if (checksum_control())
                {
                    log_parameters();
                }
            }

        } //end of if (file_opened_)
        else
        {
            if (sys_status_)
            {
                open_new_log_file();
            }

            cksum_a_ = 0.0;
            cksum_b_ = 0.0;
        }//end of else if (file_opened_)
    } //end of if (log_data_ == 1)
    else
    {
        sys_status_ = true;

        if (file_opened_)
        {
            bool succeed = console_.get_stream()->close();

            cksum_a_ = 0.0;
            cksum_b_ = 0.0;

            file_opened_ = false;
            file_init_ = false;

            if (debug_)
            {
                if (succeed)
                {
                    print_util_dbg_print("File closed\r\n");
                }
                else
                {
                    print_util_dbg_print("Error closing file\r\n");
                }
            }
        } //end of if (file_opened_)

    } //end of else (log_data_ != 1)

    return true;
}


bool Data_logging::start(void)
{
    bool success = false;

    if (!state_.is_armed())
    {
        log_data_ = true;
        success   = true;
    }

    return success;
}


bool Data_logging::stop(void)
{
    bool success = false;

    if (!state_.is_armed())
    {
        log_data_ = false;
        success   = true;
    }

    return success;
}


template<>
bool Data_logging::add_field(const uint8_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_UINT8_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const int8_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_INT8_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const uint16_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_UINT16_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const int16_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_INT16_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const uint32_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_UINT32_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const int32_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_INT32_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const uint64_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_UINT64_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const int64_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_INT64_T;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const float* val, const char* param_name, uint32_t precision)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = (double*) val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_FLOAT;
                new_param->precision                 = precision;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const double* val, const char* param_name, uint32_t precision)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (data_logging_count_ < config_.max_data_logging_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                data_logging_entry_t* new_param = &data_log_[data_logging_count_];

                new_param->param                     = val;
                strcpy(new_param->param_name,        param_name);
                new_param->data_type                 = MAVLINK_TYPE_DOUBLE;
                new_param->precision                 = precision;

                data_logging_count_ += 1;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


template<>
bool Data_logging::add_field(const bool* val, const char* param_name)
{
    bool add_success = true;

    add_success = add_field((uint8_t*)val, param_name);

    return add_success;
}
