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
 * \file    matrix.hpp
 *
 * \author  MAV'RIC Team
 * \author  Julien Lecoeur
 *
 * \brief   Templated matrix library
 *
 ******************************************************************************/

#include "util/matrix.hpp"

namespace mat
{


/**
 * Inversion of 1x1 matrix (closed form)
 */
template<>
bool op::inverse(const Mat<1,1,float>& m, Mat<1,1,float>& res)
{
    float det = m.d[0];
    
    if (det == 0)
    {
        return false;
    }

    res[0] = 1.0f / det;

    return true;
}


/**
 * Inversion of 2x2 matrix (closed form)
 */
template<>
bool op::inverse(const Mat<2,2,float>& m, Mat<2,2,float>& res)
{
    float det = m.d[0] * m.d[3] - m.d[1] * m.d[2];
    
    if (det == 0)
    {
        return false;
    }

    res = std::array<float,4>{{  m.d[3] / det, -m.d[1] / det,
                                -m.d[2] / det,  m.d[0] / det  }};

    return true;
}


/**
 * Inversion of 3x3 matrix (closed form)
 */
template<>
bool op::inverse(const Mat<3,3,float>& m, Mat<3,3,float>& res)
{
    float det = m.d[0]*(m.d[4]*m.d[8] - m.d[5]*m.d[7]) - m.d[1]*(m.d[3]*m.d[8] - m.d[5]*m.d[6]) + m.d[2]*(m.d[3]*m.d[7] - m.d[4]*m.d[6]);
    
    if (det == 0)
    {
        return false;
    }

    res = std::array<float,9>{{  (m.d[4]*m.d[8] - m.d[5]*m.d[7]) / det, -(m.d[1]*m.d[8] - m.d[2]*m.d[7]) / det, (m.d[1]*m.d[5] - m.d[2]*m.d[4]) / det,
                                -(m.d[3]*m.d[8] - m.d[5]*m.d[6]) / det, (m.d[0]*m.d[8] - m.d[2]*m.d[6]) / det, -(m.d[0]*m.d[5] - m.d[2]*m.d[3]) / det,
                                (m.d[3]*m.d[7] - m.d[4]*m.d[6]) / det, -(m.d[0]*m.d[7] - m.d[1]*m.d[6]) / det, (m.d[0]*m.d[4] - m.d[1]*m.d[3]) / det  }};

    return true;
}


}