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
 * \file    matrix.hxx
 *
 * \author  MAV'RIC Team
 * \author  Julien Lecoeur
 *
 * \brief   Templated matrix library
 *
 ******************************************************************************/


#ifndef MATRIX_HXX__
#define MATRIX_HXX__


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T>::Mat(T value, bool diag)
{
    if (diag == false)
    {
        for (uint32_t i = 0; i < N*P; ++i)
        {
            d[i] = value;
        }
    }
    else
    {
        for (uint32_t i = 0; i < N; ++i)
        {
            for (uint32_t j = 0; j < P; ++j)
            {
                uint32_t index = i*P + j;
                if (i==j)
                {
                    d[index] = value;
                }
                else
                {
                    d[index] = 0.0f;
                }
            }
        }
    }
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T>::Mat(const Mat<N,P>& mat)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        d[i] = mat.d[i];
    }
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T>::Mat(const std::array<T,N*P> arr)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        d[i] = arr[i];
    }
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T>::Mat(const std::initializer_list<T> list)
{
    Mat();

    uint32_t size = list.size();
    if(size > 0)
    {
        if (size > N*P)
        {
            size = N*P;
        }

        const T* it = list.begin();

        for (uint32_t i = 0; i < size; ++i)
        {
            d[i] = *it;
            ++it;
        }
    }
}


template<uint32_t N, uint32_t P, typename T>
uint32_t Mat<N,P,T>::rows(void)
{
    return N;
}


template<uint32_t N, uint32_t P, typename T>
uint32_t Mat<N,P,T>::cols(void)
{
    return P;
}


template<uint32_t N, uint32_t P, typename T>
uint32_t Mat<N,P,T>::index(uint32_t i, uint32_t j)
{
    return i * P + j;
}


template<uint32_t N, uint32_t P, typename T>
const T& Mat<N,P,T>::operator()(const uint32_t& i, const uint32_t& j) const
{
    if (i < N && j < P)
    {
        return d[i * P + j];
    }
    else
    {
        return d[0];
    }
}


template<uint32_t N, uint32_t P, typename T>
T& Mat<N,P,T>::operator()(const uint32_t& i, const uint32_t& j)
{
    if (i < N && j < P)
    {
        return d[i * P + j];
    }
    else
    {
        return d[0];
    }
}


template<uint32_t N, uint32_t P, typename T>
const T& Mat<N,P,T>::operator[](const uint32_t& index) const
{
    if (index < N * P)
    {
        return d[index];
    }
    else
    {
        return d[0];
    }
}


template<uint32_t N, uint32_t P, typename T>
T& Mat<N,P,T>::operator[](const uint32_t& index)
{
    if (index < N * P)
    {
        return d[index];
    }
    else
    {
        return d[0];
    }
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::add(const Mat& m) const
{
    Mat res;
    mat::op::add(*this, m, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::add(const T value) const
{
    Mat res;
    mat::op::add(*this, value, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::operator+(const Mat& m) const
{
    return add(m);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::operator+(T value) const
{
    Mat res = add(value);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> operator+(const T value, const Mat<N,P,T>& m)
{
    return m.add(value);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::add_inplace(const Mat& m)
{
    mat::op::add(*this, m, *this);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::add_inplace(const T value)
{
    mat::op::add(*this, value, *this);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::operator+=(const Mat& m)
{
    add_inplace(m);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::operator+=(const T value)
{
    add_inplace(value);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::subtract(const Mat& m) const
{
    Mat res;
    mat::op::subtract(*this, m, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::subtract(const T value) const
{
    Mat res;
    mat::op::subtract(*this, value, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::operator-(const Mat& m) const
{
    return subtract(m);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::operator-(const T value) const
{
    return subtract(value);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> operator-(const T value, const Mat<N,P,T>& m)
{
    Mat<N,P,T> m1(value);
    return m1.subtract(m);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::subtract_inplace(const Mat& m)
{
    mat::op::subtract(*this, m, *this);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::subtract_inplace(const T value)
{
    mat::op::subtract(*this, value, *this);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::operator-=(const Mat& m)
{
    subtract_inplace(m);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::operator-=(const T value)
{
    subtract_inplace(value);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::multiply(const Mat& m) const
{
    Mat res;
    mat::op::multiply(*this, m, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::multiply(const T value) const
{
    Mat res;
    mat::op::multiply(*this, value, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::operator*(const Mat& m) const
{
    return multiply(m);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::operator*(const T value) const
{
    return multiply(value);
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> operator*(const T value, const Mat<N,P,T>& m)
{
    return m.multiply(value);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::multiply_inplace(const Mat& m)
{
    mat::op::multiply(*this, m, *this);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::multiply_inplace(const T value)
{
    mat::op::multiply(*this, value, *this);
}


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::operator*=(const Mat& m)
{
    multiply_inplace(m);
};


template<uint32_t N, uint32_t P, typename T>
void Mat<N,P,T>::operator*=(const T value)
{
    multiply_inplace(value);
};


template<uint32_t N, uint32_t P, typename T>
Mat<P,N,T> Mat<N,P,T>::transpose(void) const
{
    Mat<P,N,T> res;
    mat::op::transpose(*this, res);
    return res;
};


template<uint32_t N, uint32_t P, typename T>
Mat<P,N,T> Mat<N,P,T>::operator~(void) const
{
    return transpose();
};


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::inverse(bool& success) const
{
    Mat<N,N,T> res;
    success = mat::op::inverse(*this, res);
    return res;
}


template<uint32_t N, uint32_t P, typename T>
Mat<N,P,T> Mat<N,P,T>::inv(bool& success) const
{
    return inverse(success);
}


namespace mat
{

template<uint32_t N, uint32_t P, typename T>
void op::add(const Mat<N,P,T>& m1, const Mat<N,P,T>& m2, Mat<N,P,T>& res)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        res.d[i] = m1.d[i] + m2.d[i];
    }
}


template<uint32_t N, uint32_t P, typename T>
void op::add(const Mat<N,P,T>& m1, const T value, Mat<N,P,T>& res)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        res.d[i] = m1.d[i] + value;
    }
}


template<uint32_t N, uint32_t P, typename T>
void op::subtract(const Mat<N,P,T>& m1, const Mat<N,P,T>& m2, Mat<N,P,T>& res)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        res.d[i] = m1.d[i] - m2.d[i];
    }
}


template<uint32_t N, uint32_t P, typename T>
void op::subtract(const Mat<N,P,T>& m1, const T value, Mat<N,P,T>& res)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        res.d[i] = m1.d[i] - value;
    }
}


template<uint32_t N, uint32_t P, typename T>
void op::multiply(const Mat<N,P,T>& m1, const Mat<N,P,T>& m2, Mat<N,P,T>& res)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        res.d[i] = m1.d[i] * m2.d[i];
    }
}


template<uint32_t N, uint32_t P, typename T>
void op::multiply(const Mat<N,P,T>& m1, const T value, Mat<N,P,T>& res)
{
    for (uint32_t i = 0; i < N*P; ++i)
    {
        res.d[i] = m1.d[i] * value;
    }
}


template<uint32_t N, uint32_t P, typename T>
void op::transpose(const Mat<N,P,T>& m, Mat<P,N,T>& res)
{
    uint32_t index  = 0;
    uint32_t index2 = 0;

    for (uint32_t i = 0; i < N; ++i)
    {
        for (uint32_t j = 0; j < P; ++j)
        {
            index  = i*P + j;
            index2 = j*N + i;
            res.d[index2] = m.d[index];
        }
    }
}


template<uint32_t N, uint32_t P, uint32_t Q, typename T>
void op::dot(const Mat<N,P,T>& m1, const Mat<P,Q,T>& m2, Mat<N,Q,T>& res)
{
    for (uint32_t i = 0; i < N; ++i)
    {
        for (uint32_t j = 0; j < Q; ++j)
        {
            res.d[i*Q+j] = 0.0f;

            for (uint32_t k = 0; k < P; ++k)
            {
                res.d[i*Q+j] += m1.d[i*P + k] * m2.d[k*Q+j];
            }
        }
    }
}


/**
 * TODO
 * General matrix inversion using Gauss-Jordan reduction
 */
// template<uint32_t N, typename T>
// bool op::inverse(const Mat<N,N,T>& m, Mat<N,N,T>& res)
// {
    // Mat<N,2*N,T> aug;

    // for (uint32_t i=0; i < N; i++)
    // {
    //     for (uint32_t j=0; j < N; j++)
    //     {
    //         float v = m.d[i * N + j];

    //         //  clean floats
    //         float precision = 1.0e-6f;
    //         if (-precision < v && v < precision)
    //         {
    //             v = 0.0f;
    //         }

    //         aug(i,j) = v;
    //         aug(i,N+j) = (i == j) * 1.0f;    //  right hand side is I3
    //     }
    // }

    // //  reduce
    // for (uint32_t i=0; i < N; i++)   //  iterate over rows
    // {
    //     //  look for a pivot
    //     float p = aug(i,i);

    //     if (p == 0.0f)
    //     {
    //         bool pFound = false;
    //         for (uint32_t k=i+1; k < N; k++)
    //         {
    //             p = aug(k,i);
    //             if (p != 0.0f)
    //             {
    //                 //  found a pivot
    //                 aug.row_swap(i,k);
    //                 pFound = true;
    //                 break;
    //             }
    //         }

    //         if (!pFound) {
    //             //  singular, give up
    //             return false;
    //         }
    //     }

    //     //  normalize the pivot
    //     aug.row_scale(i,  1 / p);

    //     //  pivot is in right place, reduce all rows
    //     for (uint32_t k=0; k < N; k++)
    //     {
    //         if (k != i)
    //         {
    //             aug.row_madd(k, i, -aug(k,i));
    //         }
    //     }
    // }

    // return aug.template slice <N,N*2> ();
//     return  false;
// }


}

#endif /* MATRIX_HXX__ */
