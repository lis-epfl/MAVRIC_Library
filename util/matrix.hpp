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


#ifndef MATRIX_HPP__
#define MATRIX_HPP__


#include <cstdint>
#include <array>
#include <initializer_list>


/**
 * Forward declaration
 */
template<uint32_t N, uint32_t P, typename T>
class Mat;

namespace mat
{

/**
 * \brief       Perform operations on matrices
 *
 * \details     All methods are static (can be used as a namespace)
 *              This class is friend with Mat, so its methods can access private
 *              members of all specializations of Mat.
 */
class op
{
public:
    /**
     * \brief   Add two matrices
     *
     * \detail  Performs res = m1 + m2
     *
     * \detail  Can be used for in place operations
     *          (ie. m1 or m2 can be references to the same matrix as res)
     *
     * \param   m1      First matrix to add
     * \param   m2      Second matrix to add
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void add(const Mat<N,P,T>& m1, const Mat<N,P,T>& m2, Mat<N,P,T>& res);


    /**
     * \brief   Add a scalar to a matrix
     *
     * \detail  Performs res = m1 + value
     *
     * \detail  Can be used for in place operations
     *          (ie. m1 can be a reference to the same matrix as res)
     *
     * \param   m1      First matrix to add
     * \param   value   Value to add
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void add(const Mat<N,P,T>& m1, const T value, Mat<N,P,T>& res);


    /**
     * \brief   Subtract two matrices
     *
     * \detail  Performs res = m1 - m2
     *
     * \detail  Can be used for in place operations
     *          (ie. m1 or m2 can be references to the same matrix as res)
     *
     * \param   m1      First matrix to subtract
     * \param   m2      Second matrix to subtract
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void subtract(const Mat<N,P,T>& m1, const Mat<N,P,T>& m2, Mat<N,P,T>& res);


    /**
     * \brief   Subtract a scalar to a matrix
     *
     * \detail  Performs res = m1 - value
     *
     * \detail  Can be used for in place operations
     *          (ie. m1 can be a reference to the same matrix as res)
     *
     * \param   m1      First matrix to subtract
     * \param   value   Value to subtract
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void subtract(const Mat<N,P,T>& m1, const T value, Mat<N,P,T>& res);


    /**
     * \brief   Multiply two matrices element per element
     *
     * \detail  Performs res = m1 * m2 value per value
     *
     * \detail  Can be used for in place operations
     *          (ie. m1 or m2 can be references to the same matrix as res)
     *
     * \param   m1      First matrix to multiply
     * \param   m2      Second matrix to multiply
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void multiply(const Mat<N,P,T>& m1, const Mat<N,P,T>& m2, Mat<N,P,T>& res);


    /**
     * \brief   Multiply a scalar to a matrix
     *
     * \detail  Performs res = m1 * value
     *
     * \detail  Can be used for in place operations
     *          (ie. m1 can be a reference to the same matrix as res)
     *
     * \param   m1      First matrix to multiply
     * \param   value   Value to multiply
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void multiply(const Mat<N,P,T>& m1, const T value, Mat<N,P,T>& res);


    /**
     * \brief   Transpose a matrix
     *
     * \detail  Performs res = m.T
     *
     * \detail  Warning! Can NOT be used for in place operations
     *
     * \param   m1      Matrix to transpose
     * \param   res     Result
     *
     * \tparam  N       Number of rows
     * \tparam  P       Number of columns
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, typename T>
    static void transpose(const Mat<N,P,T>& m, Mat<P,N,T>& res);


    /**
     * \brief   Matrix dot product
     *
     * \detail  Performs res = m1 . m2
     *
     * \detail  Warning! Can NOT be used for in place operations
     *
     * \param   m1      First matrix to multiply
     * \param   m2      Second matrix to multiply
     * \param   res     Result
     *
     * \tparam  N       Number of rows of 1st matrix , also number of rows of output matrix
     * \tparam  P       Number of columns of 1st matrix , also number of rows of 2nd matrix
     * \tparam  Q       Number of columns of 2nd matrix , also number of columns of output matrix
     * \tparam  T       Type of data
     */
    template<uint32_t N, uint32_t P, uint32_t Q, typename T>
    static void dot(const Mat<N,P,T>& m1, const Mat<P,Q,T>& m2, Mat<N,Q,T>& res);


    /**
     * \brief   Matrix dot product
     *
      * \detail  Performs res = m1 . m2
     *
     * \detail  Warning! Can NOT be used for in place operations
     *
     * \param   m           Square matrix to inverse
     * \param   res         Result
     *
     * \tparam  N       Number of rows
     * \tparam  T       Type of data
     *
     * \return  success     Indicates if the matrix could be inverted (true for success)
     */
    template<uint32_t N, typename T>
    static bool inverse(const Mat<N,N,T>& m, Mat<N,N,T>& res);


    /**
     * \brief   Insert a sub matrix into another matrix
     *
      * \detail  Insert matrix m2 into m1, at starting index (i_start,j_start)
     *
     * \detail  Warning! Can NOT be used for in place operations
     *
     * \param   m1          Matrix in which the other matrix will be inserted
     * \param   m2          Matrix which will be inserted
     * \param   i_start     Row index of the first top left element to be inserted (index in m1)
     * \param   j_start     Column index of the first top left element to be inserted (index in m1)
     * \param   res         Result
     *
     * \tparam  N       Number of rows of 1st matrix , also number of rows of output matrix
     * \tparam  P       Number of columns of 1st matrix , also number of rows of 2nd matrix
     * \tparam  Q       Number of columns of 2nd matrix , also number of columns of output matrix
     * \tparam  T       Type of data
     *
     * \return  success     Return true if the matrix was inserted correctly, false if it was too big (in this case only the part that fitted was inserted)
     */
    template<uint32_t M, uint32_t N, uint32_t P, uint32_t Q, typename T>
    static bool insert(const Mat<M,N,T>& m1, const Mat<P,Q,T>& m2, uint32_t i_start, uint32_t j_start, Mat<M,N,T>& res);
};

}


/**
 * \brief   Templated matrix class
 *
 * \tparam  N   Number of rows
 * \tparam  P   Number of columns
 * \tparam  T   Type of data (default: float)
 */
template<uint32_t N, uint32_t P, typename T=float>
class Mat
{
friend class mat::op;   ///< Friend with op class, so that methods of op can access
                        ///< private members of Mat<N,P,T>

public:
    /**
     * \brief   Default constructor
     *
     * \param   value   Value used to fill the matrix
     * \param   diag    Flag indicating if the matrix is diagonal
     */
    Mat(T value=0.0f, bool diag=false);

    /**
     * \brief   Copy constructor
     *
     * \param   mat     Matrix to copy
     */
    Mat(const Mat<N,P>& mat);


    /**
     * \brief   Constructor from array
     *
     * \details Compile-time size check, does not work on avr32
     *
     * \param   array     Array of values
     */
    Mat(const std::array<T,N*P> arr);


    /**
     * \brief   Constructor from list
     *
     * \details Runtime size check, works on avr32
     *
     * \param   array     Array of values
     */
    Mat(const std::initializer_list<T> list);


    /**
     * \brief   Get number of rows
     *
     * \return  N
     */
    uint32_t rows(void);


    /**
     * \brief   Get number of columns
     *
     * \return  P
     */
    uint32_t cols(void);


    /**
     * \brief   Return index in internal data array from row/column
     *
     * \return  index
     */
    uint32_t index(uint32_t i, uint32_t j);

    /**
     * \brief       Safe access to matrix element
     *
     * \param   i   Row
     * \param   j   Column
     *
     * \return      M(i,j) when indices are correct, M(0,0) when indices overflow
     */
    const T& operator()(const uint32_t& i, const uint32_t& j) const;
    T& operator()(const uint32_t& i, const uint32_t& j);


    /**
     * \brief           Direct access to matrix data
     *
     * \param   index   Index in array (index=i*P+j for i-th row and j-th column)
     *
     * \return          M[index] when index is correct, M(0,0) when index overflows
     */
    const T& operator[](const uint32_t& index) const;
    T& operator[](const uint32_t& index);


    /**
     * \brief   Add to a matrix
     *
     * \param   m   Matrix to add
     *
     * \return  result
     */
    Mat add(const Mat& m) const;


    /**
     * \brief   Add to a scalar
     *
     * \param   value   Scalar to add
     *
     * \return  result
     */
    Mat add(const T value) const;


    /**
     * \brief   Add to a matrix
     *
     * \param   m   Matrix to add
     *
     * \return  result
     */
    Mat operator+(const Mat& m) const;


    /**
     * \brief   Add to a scalar
     *
     * \param   value   Scalar to add
     *
     * \return  result
     */
    Mat operator+(T value) const;


    /**
     * \brief   Add a scalar to the matrix (ex: value + m)
     *
     * \param   value   Scalar to add
     * \param   m       Scalar to add
     *
     * \return  result
     */
    // template<uint32_t NN, uint32_t PP, typename TT>
    template<typename TT>
    friend Mat operator+(const T value, const Mat& m);


    /**
     * \brief   Increment by a matrix (inplace operation)
     *
     * \param   m   Matrix to add
     *
     * \return  result
     */
    void add_inplace(const Mat& m);


    /**
     * \brief   Increment by a scalar (inplace operation)
     *
     * \param   value   Scalar to add
     *
     * \return  result
     */
    void add_inplace(const T value);


    /**
     * \brief   Increment by a matrix (inplace operation)
     *
     * \param   m   Matrix to add
     *
     * \return  result
     */
    void operator+=(const Mat& m);


    /**
     * \brief   Increment by a scalar (inplace operation)
     *
     * \param   value   Scalar to add
     *
     * \return  result
     */
    void operator+=(const T value);


    /**
     * \brief   Subtract a matrix
     *
     * \param   m   Matrix to subtract
     *
     * \return  result
     */
    Mat subtract(const Mat& m) const;


    /**
     * \brief   Subtract a scalar
     *
     * \param   value   Scalar to subtract
     *
     * \return  result
     */
    Mat subtract(const T value) const;


    /**
     * \brief   Subtract a matrix
     *
     * \param   m   Matrix to subtract
     *
     * \return  result
     */
    Mat operator-(const Mat& m) const;


    /**
     * \brief   Subtract a scalar
     *
     * \param   value   Scalar to subtract
     *
     * \return  result
     */
    Mat operator-(const T value) const;


    /**
     * \brief   Subtract a matrix to a scalar (ex: value - m)
     *
     * \param   value   Scalar to subtract
     *
     * \return  result
     */
    // template<uint32_t NN, uint32_t PP, typename TT>
    template<typename TT>
    friend Mat operator-(const T value, const Mat& m);


    /**
     * \brief   Decrement by a matrix (inplace operation)
     *
     * \param   m   Matrix to subtract
     *
     * \return  result
     */
    void subtract_inplace(const Mat& m);


    /**
     * \brief   Decrement by a scalar (inplace operation)
     *
     * \param   value   Scalar to subtract
     *
     * \return  result
     */
    void subtract_inplace(const T value);


    /**
     * \brief   Decrement by a matrix (inplace operation)
     *
     * \param   m   Matrix to subtract
     *
     * \return  result
     */
    void operator-=(const Mat& m);


    /**
     * \brief   Decrement by a scalar (inplace operation)
     *
     * \param   value   Scalar to subtract
     *
     * \return  result
     */
    void operator-=(const T value);

    /**
     * \brief   Multiply by a matrix
     *
     * \param   m   Matrix to multiply
     *
     * \return  result
     */
    Mat multiply(const Mat& m) const;


     /**
     * \brief   Multiply by a scalar
     *
     * \param   value   Scalar to multiply
     *
     * \return  result
     */
    Mat multiply(const T value) const;


    /**
     * \brief   Multiply by a matrix
     *
     * \param   m   Matrix to multiply
     *
     * \return  result
     */
    Mat operator*(const Mat& m) const;


     /**
     * \brief   Multiply by a scalar
     *
     * \param   value   Scalar to multiply
     *
     * \return  result
     */
    Mat operator*(const T value) const;


     /**
     * \brief   Multiply a scalar to a matrix (ex: value * m)
     *
     * \param   value   Scalar to multiply
     * \param   m       Matrix to multiply
     *
     * \return  result
     */
    template<typename TT>
    friend Mat operator*(const T value, const Mat& m);


    /**
     * \brief   Multiply by a matrix (inplace operation)
     *
     * \param   m   Matrix to multiply
     *
     * \return  result
     */
    void multiply_inplace(const Mat& m);


    /**
     * \brief   Multiply by a scalar (inplace operation)
     *
     * \param   value   Scalar to multiply
     *
     * \return  result
    */
    void multiply_inplace(const T value);


    /**
     * \brief   Multiply by a matrix (inplace operation)
     *
     * \param   m   Matrix to multiply
     *
     * \return  result
     */
    void operator*=(const Mat& m);


    /**
     * \brief   Multiply by a scalar (inplace operation)
     *
     * \param   value   Scalar to multiply
     *
     * \return  result
    */
    void operator*=(const T value);


    /**
     * \brief   Dot product
     *
     * \detail  For some reason, there is a compilation error if this method is
     *          defined in matrix.hxx
     *
     * \param   m   Matrix to dot-multiply
     *
     * \tparam  Q   Number of columns of 2nd matrix, also number of columns of result
     *
     * \return  result
     */
    template<uint32_t Q>
    Mat<N,Q,T> dot(const Mat<P,Q,T>& m) const
    {
        Mat<N,Q,T> res;
        mat::op::dot(*this, m, res);
        return res;
    }


    /**
     * \brief  Dot product
     *
     * \detail  For some reason, there is a compilation error if this method is
     *          defined in matrix.hxx
     *
     * \param   m   Matrix to dot-multiply
     *
     * \tparam  Q   Number of columns of 2nd matrix, also number of columns of result
     *
     * \return  result
     */
    template<uint32_t Q>
    Mat<N,Q,T> operator%(const Mat<P,Q,T>& m) const
    {
        return dot(m);
    }


    /**
     * \brief  Get transposed matrix
     *
     * \return  result
     */
    Mat<P,N,T> transpose(void) const;


    /**
     * \brief  Get transposed matrix
     *
     * \return  result
     */
    Mat<P,N,T> operator~(void) const;


    /**
     * \brief  Get inversed matrix
     *
     * \param   success     Indicates if the matrix could be inverted
     *
     * \return  result
     */
    Mat inverse(bool& success) const;


    /**
     * \brief  Get inversed matrix
     *
     * \param   success     Indicates if the matrix could be inverted
     *
     * \return  result
     */
    Mat inv(bool& success) const;


    /**
     * \brief  Insert a matrix into this one
     *
     * \param   m           Matrix to be inserted
     * \param   i           Row index of the first element where the matrix should be inserted
     * \param   j           Column index of the first element where the matrix should be inserted
     * \param   success     False if the inserted matrix was to big to fit (in this case, the elements that fits are still inserted)
     *
     * \return  result
     */
    // template<uint32_t M, uint32_t Q>
    // Mat insert(Mat<M,Q,T> m, uint32_t i, uint32_t j, bool& success) const;


    /**
     * \brief  Insert a matrix into this one
     *
     * \param   m           Matrix to be inserted
     * \param   i           Row index of the first element where the matrix should be inserted
     * \param   j           Column index of the first element where the matrix should be inserted
     *
     * \return  result
     */
    // template<uint32_t M, uint32_t Q>
    // Mat insert(Mat<M,Q,T> m, uint32_t i, uint32_t j) const;


private:
    std::array<T,N*P> d;   ///< Data
};


/**
 *  3D Vector
 */
typedef Mat<3, 1, float> Vector3f;


#include "matrix.hxx"


#endif /* MATRIX_HPP__ */
