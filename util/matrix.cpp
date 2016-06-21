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


/**
 * Inversion of 4x4 matrix (closed form)
 *
 * Equations from www.cg.info.hiroshima-cu.ac.jp/~miyataki/knowledge/teche23.html
 */
template<>
bool op::inverse(const Mat<4,4,float>& m, Mat<4,4,float>& res)
{
    float a11 = m(0,0);
    float a12 = m(0,1);
    float a13 = m(0,2);
    float a14 = m(0,3);
    float a21 = m(1,0);
    float a22 = m(1,1);
    float a23 = m(1,2);
    float a24 = m(1,3);
    float a31 = m(2,0);
    float a32 = m(2,1);
    float a33 = m(2,2);
    float a34 = m(2,3);
    float a41 = m(3,0);
    float a42 = m(3,1);
    float a43 = m(3,2);
    float a44 = m(3,3);
    float det =
         a11*a22*a33*a44 + a11*a23*a34*a42 + a11*a24*a32*a43
        +a12*a21*a34*a43 + a12*a23*a31*a44 + a12*a24*a33*a41
        +a13*a21*a32*a44 + a13*a22*a34*a41 + a13*a24*a31*a42
        +a14*a21*a33*a42 + a14*a22*a31*a43 + a14*a23*a32*a41
        -a11*a22*a34*a43 - a11*a23*a32*a44 - a11*a24*a33*a42
        -a12*a21*a33*a44 - a12*a23*a34*a41 - a12*a24*a31*a43
        -a13*a21*a34*a42 - a13*a22*a31*a44 - a13*a24*a32*a41
        -a14*a21*a32*a43 - a14*a22*a33*a41 - a14*a23*a31*a42;

    if (det == 0)
    {
        return false;
    }

    float b11 = a22*a33*a44 + a23*a34*a42 + a24*a32*a43 - a22*a34*a43 - a23*a32*a44 - a24*a33*a42;
    float b12 = a12*a34*a43 + a13*a32*a44 + a14*a33*a42 - a12*a33*a44 - a13*a34*a42 - a14*a32*a43;
    float b13 = a12*a23*a44 + a13*a24*a42 + a14*a22*a43 - a12*a24*a43 - a13*a22*a44 - a14*a23*a42;
    float b14 = a12*a24*a33 + a13*a22*a34 + a14*a23*a32 - a12*a23*a34 - a13*a24*a32 - a14*a22*a33;
    float b21 = a21*a34*a43 + a23*a31*a44 + a24*a33*a41 - a21*a33*a44 - a23*a34*a41 - a24*a31*a43;
    float b22 = a11*a33*a44 + a13*a34*a41 + a14*a31*a43 - a11*a34*a43 - a13*a31*a44 - a14*a33*a41;
    float b23 = a11*a24*a43 + a13*a21*a44 + a14*a23*a41 - a11*a23*a44 - a13*a24*a41 - a14*a21*a43;
    float b24 = a11*a23*a34 + a13*a24*a31 + a14*a21*a33 - a11*a24*a33 - a13*a21*a34 - a14*a23*a31;
    float b31 = a21*a32*a44 + a22*a34*a41 + a24*a31*a42 - a21*a34*a42 - a22*a31*a44 - a24*a32*a41;
    float b32 = a11*a34*a42 + a12*a31*a44 + a14*a32*a41 - a11*a32*a44 - a12*a34*a41 - a14*a31*a42;
    float b33 = a11*a22*a44 + a12*a24*a41 + a14*a21*a42 - a11*a24*a42 - a12*a21*a44 - a14*a22*a41;
    float b34 = a11*a24*a32 + a12*a21*a34 + a14*a22*a31 - a11*a22*a34 - a12*a24*a31 - a14*a21*a32;
    float b41 = a21*a33*a42 + a22*a31*a43 + a23*a32*a41 - a21*a32*a43 - a22*a33*a41 - a23*a31*a42;
    float b42 = a11*a32*a43 + a12*a33*a41 + a13*a31*a42 - a11*a33*a42 - a12*a31*a43 - a13*a32*a41;
    float b43 = a11*a23*a42 + a12*a21*a43 + a13*a22*a41 - a11*a22*a43 - a12*a23*a41 - a13*a21*a42;
    float b44 = a11*a22*a33 + a12*a23*a31 + a13*a21*a32 - a11*a23*a32 - a12*a21*a33 - a13*a22*a31;

    res(0,0) = b11 / det;
    res(0,1) = b12 / det;
    res(0,2) = b13 / det;
    res(0,3) = b14 / det;
    res(1,0) = b21 / det;
    res(1,1) = b22 / det;
    res(1,2) = b23 / det;
    res(1,3) = b24 / det;
    res(2,0) = b31 / det;
    res(2,1) = b32 / det;
    res(2,2) = b33 / det;
    res(2,3) = b34 / det;
    res(3,0) = b41 / det;
    res(3,1) = b42 / det;
    res(3,2) = b43 / det;
    res(3,3) = b44 / det;

    return true;
}

//
// TODO: implement inversion of 4x4 matrices like bellow
//
// matrix_4x4_t inv4(matrix_4x4_t m)
// {
//     matrix_4x4_t result;
//     int32_t i, j;
//     float det;

//     result.v[0][0] =    m.v[1][1]  * m.v[2][2] * m.v[3][3] -
//                         m.v[1][1]  * m.v[2][3] * m.v[3][2] -
//                         m.v[2][1]  * m.v[1][2] * m.v[3][3] +
//                         m.v[2][1]  * m.v[1][3] * m.v[3][2] +
//                         m.v[3][1]  * m.v[1][2] * m.v[2][3] -
//                         m.v[3][1]  * m.v[1][3] * m.v[2][2];

//     result.v[1][0] =   -m.v[1][0]  * m.v[2][2]  * m.v[3][3] +
//                        m.v[1][0]  * m.v[2][3]  * m.v[3][2] +
//                        m.v[2][0]  * m.v[1][2]  * m.v[3][3] -
//                        m.v[2][0]  * m.v[1][3]  * m.v[3][2] -
//                        m.v[3][0]  * m.v[1][2]  * m.v[2][3] +
//                        m.v[3][0]  * m.v[1][3]  * m.v[2][2];

//     result.v[2][0] =    m.v[1][0]  * m.v[2][1] * m.v[3][3] -
//                         m.v[1][0]  * m.v[2][3] * m.v[3][1] -
//                         m.v[2][0]  * m.v[1][1] * m.v[3][3] +
//                         m.v[2][0]  * m.v[1][3] * m.v[3][1] +
//                         m.v[3][0]  * m.v[1][1] * m.v[2][3] -
//                         m.v[3][0]  * m.v[1][3] * m.v[2][1];

//     result.v[3][0] =   -m.v[1][0]  * m.v[2][1] * m.v[3][2] +
//                        m.v[1][0]  * m.v[2][2] * m.v[3][1] +
//                        m.v[2][0]  * m.v[1][1] * m.v[3][2] -
//                        m.v[2][0]  * m.v[1][2] * m.v[3][1] -
//                        m.v[3][0]  * m.v[1][1] * m.v[2][2] +
//                        m.v[3][0]  * m.v[1][2] * m.v[2][1];

//     result.v[0][1] =   -m.v[0][1]  * m.v[2][2] * m.v[3][3] +
//                        m.v[0][1]  * m.v[2][3] * m.v[3][2] +
//                        m.v[2][1]  * m.v[0][2] * m.v[3][3] -
//                        m.v[2][1]  * m.v[0][3] * m.v[3][2] -
//                        m.v[3][1]  * m.v[0][2] * m.v[2][3] +
//                        m.v[3][1]  * m.v[0][3] * m.v[2][2];

//     result.v[1][1] =    m.v[0][0]  * m.v[2][2] * m.v[3][3] -
//                         m.v[0][0]  * m.v[2][3] * m.v[3][2] -
//                         m.v[2][0]  * m.v[0][2] * m.v[3][3] +
//                         m.v[2][0]  * m.v[0][3] * m.v[3][2] +
//                         m.v[3][0]  * m.v[0][2] * m.v[2][3] -
//                         m.v[3][0]  * m.v[0][3] * m.v[2][2];

//     result.v[2][1] =   -m.v[0][0]  * m.v[2][1] * m.v[3][3] +
//                        m.v[0][0]  * m.v[2][3] * m.v[3][1] +
//                        m.v[2][0]  * m.v[0][1] * m.v[3][3] -
//                        m.v[2][0]  * m.v[0][3] * m.v[3][1] -
//                        m.v[3][0]  * m.v[0][1] * m.v[2][3] +
//                        m.v[3][0]  * m.v[0][3] * m.v[2][1];

//     result.v[3][1] =    m.v[0][0]  * m.v[2][1] * m.v[3][2] -
//                         m.v[0][0]  * m.v[2][2] * m.v[3][1] -
//                         m.v[2][0]  * m.v[0][1] * m.v[3][2] +
//                         m.v[2][0]  * m.v[0][2] * m.v[3][1] +
//                         m.v[3][0]  * m.v[0][1] * m.v[2][2] -
//                         m.v[3][0]  * m.v[0][2] * m.v[2][1];

//     result.v[0][2] =    m.v[0][1]  * m.v[1][2] * m.v[3][3] -
//                         m.v[0][1]  * m.v[1][3] * m.v[3][2] -
//                         m.v[1][1]  * m.v[0][2] * m.v[3][3] +
//                         m.v[1][1]  * m.v[0][3] * m.v[3][2] +
//                         m.v[3][1]  * m.v[0][2] * m.v[1][3] -
//                         m.v[3][1]  * m.v[0][3] * m.v[1][2];

//     result.v[1][2] =   -m.v[0][0]  * m.v[1][2] * m.v[3][3] +
//                        m.v[0][0]  * m.v[1][3] * m.v[3][2] +
//                        m.v[1][0]  * m.v[0][2] * m.v[3][3] -
//                        m.v[1][0]  * m.v[0][3] * m.v[3][2] -
//                        m.v[3][0]  * m.v[0][2] * m.v[1][3] +
//                        m.v[3][0]  * m.v[0][3] * m.v[1][2];

//     result.v[2][2] =    m.v[0][0]  * m.v[1][1] * m.v[3][3] -
//                         m.v[0][0]  * m.v[1][3] * m.v[3][1] -
//                         m.v[1][0]  * m.v[0][1] * m.v[3][3] +
//                         m.v[1][0]  * m.v[0][3] * m.v[3][1] +
//                         m.v[3][0]  * m.v[0][1] * m.v[1][3] -
//                         m.v[3][0]  * m.v[0][3] * m.v[1][1];

//     result.v[3][2] =   -m.v[0][0]  * m.v[1][1] * m.v[3][2] +
//                        m.v[0][0]  * m.v[1][2] * m.v[3][1] +
//                        m.v[1][0]  * m.v[0][1] * m.v[3][2] -
//                        m.v[1][0]  * m.v[0][2] * m.v[3][1] -
//                        m.v[3][0]  * m.v[0][1] * m.v[1][2] +
//                        m.v[3][0]  * m.v[0][2] * m.v[1][1];

//     result.v[0][3] =   -m.v[0][1] * m.v[1][2] * m.v[2][3] +
//                        m.v[0][1] * m.v[1][3] * m.v[2][2] +
//                        m.v[1][1] * m.v[0][2] * m.v[2][3] -
//                        m.v[1][1] * m.v[0][3] * m.v[2][2] -
//                        m.v[2][1] * m.v[0][2] * m.v[1][3] +
//                        m.v[2][1] * m.v[0][3] * m.v[1][2];

//     result.v[1][3] =    m.v[0][0] * m.v[1][2] * m.v[2][3] -
//                         m.v[0][0] * m.v[1][3] * m.v[2][2] -
//                         m.v[1][0] * m.v[0][2] * m.v[2][3] +
//                         m.v[1][0] * m.v[0][3] * m.v[2][2] +
//                         m.v[2][0] * m.v[0][2] * m.v[1][3] -
//                         m.v[2][0] * m.v[0][3] * m.v[1][2];

//     result.v[2][3] =    -m.v[0][0] * m.v[1][1] * m.v[2][3] +
//                         m.v[0][0] * m.v[1][3] * m.v[2][1] +
//                         m.v[1][0] * m.v[0][1] * m.v[2][3] -
//                         m.v[1][0] * m.v[0][3] * m.v[2][1] -
//                         m.v[2][0] * m.v[0][1] * m.v[1][3] +
//                         m.v[2][0] * m.v[0][3] * m.v[1][1];

//     result.v[3][3] =    m.v[0][0] * m.v[1][1] * m.v[2][2] -
//                         m.v[0][0] * m.v[1][2] * m.v[2][1] -
//                         m.v[1][0] * m.v[0][1] * m.v[2][2] +
//                         m.v[1][0] * m.v[0][2] * m.v[2][1] +
//                         m.v[2][0] * m.v[0][1] * m.v[1][2] -
//                         m.v[2][0] * m.v[0][2] * m.v[1][1];

//     det =   m.v[0][0] * result.v[0][0]
//             +   m.v[0][1] * result.v[1][0]
//             +   m.v[0][2] * result.v[2][0]
//             +   m.v[0][3] * result.v[3][0];

//     if (det == 0)
//     {
//         print_util_dbg_print("inversion 4x4 failed\r\n");
//         return zero_4x4;
//     }

//     det = 1.0f / det;

//     for (i = 0; i < 4; i++)
//     {
//         for (j = 0; j < 4; j++)
//         {
//             result.v[i][j] = result.v[i][j] * det;
//         }
//     }
//     return result;
// }

}
