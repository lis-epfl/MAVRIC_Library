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
 * \file data_logging.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Performs the data logging on a file
 *
 ******************************************************************************/


#ifndef DATA_LOGGING_H__
#define DATA_LOGGING_H__

#include "communication/mavlink_communication.hpp"
#include "communication/state.hpp"
#include "hal/common/file.hpp"
#include "hal/common/console.hpp"

/**
 * \brief   Structure of data logging parameter.
 */
typedef struct
{
    const double* param;                                              ///< Pointer to the parameter value
    char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];  ///< Parameter name composed of 16 characters
    mavlink_message_type_t data_type;                           ///< Parameter type
    uint8_t precision;                                          ///< Number of digit after the zero
} data_logging_entry_t;


/**
 * \brief       Configuration of the data_logging element
 */
typedef struct
{
    uint32_t max_data_logging_count;            ///< Maximum number of parameters
    uint16_t max_logs;                          ///< The max number of logged files with the same name on the SD card
    bool debug;                                 ///< Indicates if debug messages should be printed for each param change
    uint32_t log_data;                          ///< The initial state of writing a file
} data_logging_conf_t;

/**
 * \brief   Default configuration for the data_logging
 *
 * \return  Config structure
 */
static inline data_logging_conf_t data_logging_default_config();

/**
 * \brief   The class of the log of the data
 *
 * \details     data_logging_set is implemented as pointer because its memory will be
 *              allocated during initialisation
 */
class Data_logging
{
public:
    /**
     * \brief   Data logging constructor
     */
    Data_logging(File& file, State& state, data_logging_conf_t config = data_logging_default_config());


    /**
     * \brief   Initialise the data logging module
     *
     * \param   file_name               The pointer to name of the file to create
     * \param   continuous_write        Boolean to state whether writing should be continous or not
     * \param   sysid                   The system identification number of the MAV
     *
     * \return  True if the init succeed, false otherwise
     */
    bool create_new_log_file(const char* file_name_, bool continuous_write_, uint32_t sysid);

    /**
     * \brief   The task to log the data to the SD card
     *
     * \param   data_logging            The pointer to the data logging structure
     *
     * \return  The result of the task execution
     */
    bool update();


    /**
     * \brief   Start logging data
     * 
     * \detail  Will succeed only if the MAV is disarmed
     * 
     * \return  Success
     */
    bool start(void);


    /**
     * \brief   Stop logging data
     * 
     * \detail  Will succeed only if the MAV is disarmed
     * 
     * \return  Success
     */
    bool stop(void);


    /**
     * \brief   Registers parameter to log on the SD card (integer version)
     *
     * \param   data_logging            The pointer to the data logging structure
     * \param   val                     The parameter value
     * \param   param_name              Name of the parameter
     *
     * \tparam  T                       Type of parameter to add
     * 
     * \return  True if the parameter was added, false otherwise
     */
    template<typename T>
    bool add_field(const T* val, const char* param_name);


    /**
     * \brief   Registers parameter to log on the SD card (float and double version)
     *
     * \param   data_logging            The pointer to the data logging structure
     * \param   val                     The parameter value
     * \param   param_name              Name of the parameter
     * \param   precision               The number of digit after the zero
     *
     * \tparam  T                       Type of parameter to add (float or double)
     * 
     * \return  True if the parameter was added, false otherwise
     */
    template<typename T>
    bool add_field(const T* val, const char* param_name, uint32_t precision);


private:
    /**
    * \brief    Add in the file fp the first line with the name of the parameter
    *
    * \param    data_logging            The pointer to the data logging structure
    */
    void add_header_name(void);


    /**
     * \brief   Function to put a \r or a \n after the data logging parameter value (\r between them and \n and the end)
     *
     * \param   data_logging            The pointer to the data logging structure
     * \param   param_num               The index of the data logging parameter
     */
    void put_r_or_n(uint16_t param_num);


    /**
     * \brief   Function to log a new line of values
     *
     * \param   data_logging            The pointer to the data logging structure
     */
    void log_parameters(void);


    /**
     * \brief   Seek the end of an open file to append
     *
     * \param   data_logging            The pointer to the data logging structure
     */
    void seek(void);


    /**
    * \brief    Appends ".txt" to the end of a character string. If not enough
    *           memory allocated in output, will write as many letters from
    *           filename as possible, will not include .txt\0 unless entire
    *           .txt\0 can fit.
    *
    * \param    output      The output character string
    * \param    filename    The input string
    * \param    length      The maximum length of output
    *
    * \return   success     Bool stating if the entire output was written
    */
    bool filename_append_extension(char* output, char* filename, uint32_t length);


    /**
    * \brief    Appends a uint32_t to a character string with an underscore between.
    *           If not enough memory allocated in output, will write as many letters
    *           from filename as possible, will not include num\0 unless entire number
    *           and null character can fit.
    *
    * \param    output      The output character string
    * \param    filename    The input string
    * \param    num         The uint32_t to be appended to filename
    * \param    length      The maximum length of the output string, must be positive
    *
    * \return   success     Bool stating if the entire output was written
    */
    bool filename_append_int(char* output, char* filename, uint32_t num, uint32_t length);


    /**
     * \brief   Create and open a new file
     *
     * \param   data_logging            The pointer to the data logging structure
     *
     * \result  True if the file was open correctly, false otherwise
     */
    bool open_new_log_file(void);


    /**
     * \brief   Computes the checksum of data logging
     *
     * \param   data_logging            The pointer to the data logging structure
     *
     * \result  True if the the checksum of the data didn't change, false otherwise
     */
    bool checksum_control(void);

    data_logging_conf_t config_;                ///< configuration of the data_logging module
    
    bool debug_;                                 ///< Indicates if debug messages should be printed for each param change
    uint32_t data_logging_count_;               ///< Number of data logging parameter effectively in the array
    data_logging_entry_t* data_log_;            ///< Data logging array, needs memory allocation


    int buffer_name_size_;                       ///< The buffer for the size of the file's name

    char* file_name_;                            ///< The file name
    char* name_n_extension_;                     ///< Stores the name of the file

    bool file_init_;                             ///< A flag to tell whether a file is init or not
    bool file_opened_;                           ///< A flag to tell whether a file is opened or not
    bool sys_status_;                            ///< A flag to tell whether the file system status is ok or not

    bool continuous_write_;                      ///< A flag to tell whether we write continuously to the file or not
    uint32_t log_data_;                          ///< A flag to stop/start writing to file

    uint32_t logging_time_;                      ///< The time that we've passed logging since the last f_close

    double cksum_a_;                             ///< Checksum to see if the onboard parameters have changed values
    double cksum_b_;                             ///< Checksum to see if the onboard parameters have changed values

    uint32_t sys_id_;                            ///< the system ID

    Console<File> console_;                      ///< The console containing the file to write data to

    State& state_;                              ///< The pointer to the state structure
};

static inline data_logging_conf_t data_logging_default_config()
{
    data_logging_conf_t conf    = {};

    conf.max_data_logging_count = 50;
    conf.max_logs               = 500;
    conf.debug                  = true;
    conf.log_data               = 0;  // 1: log data, 0: no log data

    return conf;
};

#endif /* DATA_LOGGING_H__ */
