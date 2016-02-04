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
 * \file    dbg.hxx
 *
 * \author  MAV'RIC Team
 *
 * \brief   Write debug messages
 *
 * Prints human-readable messages to a console.
 * Before initialization, data is written to a dummy console.
 * dout() can be used for cout like syntax: 'dout() << "hello " << 123 << endl;'
 * To define '<<' operator for your own class, implement the follwing function
 * in the header of your class, outside of your class definition:
 * template<typename Writeable>
 *  Console<Writeable>& operator<< (Console<Writeable>& console, Yourclass& yourclass)
 *  {
 *      console.write(...);
 *      return console;
 *  }
 ******************************************************************************/
#ifndef DBG_HXX_
#define DBG_HXX_


namespace dbg
{

/**
 * \brief prints data to the console
 *
 * \param data  data to be printed
 *
 * \return  success
 *
 */
template<typename T>
static bool print(T data)
{
    return dout().write(data);
}


/**
 * \brief prints data to the console, adds a new line and flushes the buffer
 *
 * \param data  data to be printed
 *
 * \return  success
 *
 */
template<typename T>
bool println(T data)
{
    return dout().writeln(data);
}
};

#endif /* DBG_HXX_ */