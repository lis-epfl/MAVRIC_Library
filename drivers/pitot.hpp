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
 * \file pitot.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 * \author Nicolas Jacquemin
 *
 * \brief Abstract class for pitot tubes
 *
 ******************************************************************************/


#ifndef PITOT_HPP_
#define PITOT_HPP_


#include <array>


/**
 * \brief Abstract class for pitot tubes
 */
class Pitot
{
public:

  /**
   * \brief   Initialise the sensor
   *
   * \return  Success
   */
  virtual bool init(void) = 0;


  /**
   * \brief   Main update function
   * \detail  Reads new values from sensor
   *
   * \return  Success
   */
  virtual bool update(void) = 0;


  /**
   * \brief   Get last update time in microseconds
   *
   * \return  Update time
   */
  virtual const float& last_update_us(void) const = 0;


  /**
   * \brief   Get speed
   *
   * \detail
   *
   * \return  Value
   */
  virtual const float& pitot(void) const = 0;
};


#endif /* PITOT_HPP_ */
