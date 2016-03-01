
// 

#ifndef __MATRIX_HPP__
#define __MATRIX_HPP__

#include <array>

//#include <iostream>

extern "C"
{
#include <stdint.h>
} 

namespace mat
{
    
/**
 * Forward declarations
 */
template<uint32_t N, uint32_t P, typename T>
class Mat;


/**
 * \brief       Perform operations on matrices
 * 
 * \details     All methods are static, so this class can be used as a namespace
 *              It is friend with Mat, so the methods can access private members
 *              of all specializations of Mat.
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
     * \return  success     Indicates if the matrix could be inverted (true for success) 
     */
    template<uint32_t N, typename T>
    static bool inverse(const Mat<N,N,T>& m, Mat<N,N,T>& res);
  
    template<uint32_t N, typename T>
    static void skew(const Mat<N,1,T>& m, Mat<N,N,T>& res);  

    template<uint32_t N, typename T>
    static void norm(const Mat<N,1,T>& m, T& res);  

    template<uint32_t N, typename T>
    static void normalize(const Mat<N,1,T>& m, Mat<N,1,T>& res);  
};



template<uint32_t N, uint32_t P, typename T=float>
class Mat
{
friend class op;

public:
    /**
     * \brief   Default constructor
     * 
     * \param   value   Value used to fill the matrix
     * \param   diag    Flag indicating if the matrix is diagonal
     */
    Mat(T value=0.0f, bool diag=false)
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


    /**
     * \brief   Copy constructor
     * 
     * \param   mat     Matrix to copy
     */
    Mat(const Mat<N,P>& mat)
    {
        for (uint32_t i = 0; i < N*P; ++i)
        {
            d[i] = mat.d[i];
        }
    }


    /**
     * \brief   Constructor from array
     * 
     * \param   array     Array of values
     */
    Mat(const std::array<T,N*P> arr)
    {
        for (uint32_t i = 0; i < N*P; ++i)
        {
            d[i] = arr[i];
        }
    }


    /**
     * \brief   Display the matrix 
     */
    /*void print(void)
    {
        for (uint32_t i = 0; i < N; ++i)
        {
            std::cout << "[ "; 
            for (uint32_t j = 0; j < P; ++j)
            {
                std::cout << d[i * P + j] << " \t";
            }
            std::cout << "]" << std::endl;
        }
    }*/


    /**
     * \brief   Get number of rows
     * 
     * \return  N
     */
    uint32_t rows(void)
    {
        return N;
    }


    /**
     * \brief   Get number of columns
     * 
     * \return  P
     */
    uint32_t cols(void)
    {
        return P;
    }


    /**
     * \brief       Safe access to matrix element
     * 
     * \param   i   Row
     * \param   j   Column
     * 
     * \return      M(i,j) when indices are correct, M(0,0) when indices overflow
     */
    T& operator()(const uint32_t& i, const uint32_t& j)
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


    /**
     * \brief           Direct access to matrix data
     * 
     * \param   index   Index in array (index=i*P+j for i-th row and j-th column)
     * 
     * \return          M[index] when index is correct, M(0,0) when index overflows
     */
    T& operator[](const uint32_t& index)
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


    Mat add(const Mat& m) const
    {
        Mat res;
        op::add(*this, m, res);
        return res;
    }


    Mat add(const T value) const
    {
        Mat res;
        op::add(*this, value, res);
        return res;
    }


    Mat operator+(const Mat& m) const
    {
        return add(m);
    }


    Mat operator+(T value) const
    {
        Mat res = add(value); 
        return res;
    }


    void add_inplace(const Mat& m)
    {
        op::add(*this, m, *this);
    }


    void add_inplace(const T value)
    {
        op::add(*this, value, *this);
    }


    void operator+=(const Mat& m)
    {
        add_inplace(m);
    }


    void operator+=(const T value)
    {
        add_inplace(value);
    }


    Mat subtract(const Mat& m) const
    {
        Mat res;
        op::subtract(*this, m, res);
        return res;
    }


    Mat subtract(const T value) const
    {
        Mat res;
        op::subtract(*this, value, res);
        return res;
    }


    Mat operator-(const Mat& m) const
    {
        return subtract(m);
    }


    Mat operator-(const T value) const
    {
        return subtract(value);
    }


    void subtract_inplace(const Mat& m)
    {
        op::subtract(*this, m, *this);
    }


    void subtract_inplace(const T value)
    {
        op::subtract(*this, value, *this);
    }


    void operator-=(const Mat& m)
    {
        subtract_inplace(m);
    }


    void operator-=(const T value)
    {
        subtract_inplace(value);
    }


    Mat multiply(const Mat& m) const
    {
        Mat res;
        op::multiply(*this, m, res);
        return res;        
    }

    
    Mat multiply(const T value) const
    {
        Mat res;
        op::multiply(*this, value, res);
        return res;        
    }


    Mat operator*(const Mat& m) const
    {
        return multiply(m);
    }


    Mat operator*(const T value) const
    {
        return multiply(value);
    }


    void multiply_inplace(const Mat& m)
    {
        op::multiply(*this, m, *this);
    }

    
    void multiply_inplace(const T value)
    {
        op::multiply(*this, value, *this);
    }


    void operator*=(const Mat& m)
    {
        multiply_inplace(m);
    };


    void operator*=(const T value)
    {
        multiply_inplace(value);
    };


    template<uint32_t Q>
    Mat<N,Q,T> dot(Mat<P,Q,T> m) const
    {
        Mat<N,Q,T> res;
        op::dot(*this, m, res);
        return res;
    };


    template<uint32_t Q>
    Mat<N,Q,T> operator^(Mat<P,Q,T> m) const
    {
        return dot(m);
    };


    Mat<P,N,T> transpose(void) const
    {
        Mat<P,N,T> res;
        op::transpose(*this, res);
        return res;
    };


    Mat<P,N,T> operator~(void) const
    {
        return transpose();
    };

    

private:
    T d[N*P];   ///< Data
};

        
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


template<uint32_t N, typename T>
bool op::inverse(const Mat<N,N,T>& m, Mat<N,N,T>& res)
{    
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
    return  false;
}

    // template<uint32_t N, typename T>
    // static bool inverse(const Mat<N,N,T>& m, Mat<N,N,T>& res);

template<>
bool op::inverse(const Mat<2,2,float>& m, Mat<2,2,float>& res)
{
    float det = m.d[0] * m.d[3] - m.d[1] * m.d[2];
    
    if (det == 0)
    {
        return false;
    }

    res = std::array<float,4>{  m.d[3] / det, -m.d[1] / det,
                               -m.d[2] / det,  m.d[0] / det  };

    return true;
}

template<>
bool op::inverse(const Mat<3,3,float>& m, Mat<3,3,float>& res)
{
    float det = m.d[0]*(m.d[4]*m.d[8] - m.d[5]*m.d[7]) - m.d[1]*(m.d[3]*m.d[8] - m.d[5]*m.d[6]) + m.d[2]*(m.d[3]*m.d[7] - m.d[4]*m.d[6]);
    
    if (det == 0)
    {
        return false;
    }

    res = std::array<float,9>{  (m.d[4]*m.d[8] - m.d[5]*m.d[7]) / det, -(m.d[1]*m.d[8] - m.d[2]*m.d[7]) / det, (m.d[1]*m.d[5] - m.d[2]*m.d[4]) / det,
                                -(m.d[3]*m.d[8] - m.d[5]*m.d[6]) / det, (m.d[0]*m.d[8] - m.d[2]*m.d[6]) / det, -(m.d[0]*m.d[5] - m.d[2]*m.d[3]) / det,
                                (m.d[3]*m.d[7] - m.d[4]*m.d[6]) / det, -(m.d[0]*m.d[7] - m.d[1]*m.d[6]) / det, (m.d[0]*m.d[4] - m.d[1]*m.d[3]) / det  };

    return true;
}

template<>
void op::skew(const Mat<3,1,float>& m, Mat<3,3,float>& res)
{
    res = std::array<float,9>{  0.0f, -m.d[2], m.d[1],
                                m.d[2], 0.0f, -m.d[0],
                                -m.d[1], m.d[0], 0.0f};
}


template<uint32_t N, typename T>
void op::norm(const Mat<N,1,T>& m, T& res)
{
    res = 0.0f;

    for (uint32_t i = 0; i < N; ++i)
    {
        res += m.d[i] * m.d[i];
    }
}

template<uint32_t N, typename T>
void op::normalize(const Mat<N,1,T>& m, Mat<N,1,T>& res)
{
    T norm_vec;
    op::norm(m, norm_vec);

    for (uint32_t i = 0; i < N; ++i)
    {
        res.d[i] = m.d[i] / norm_vec;
    }
}

}

#endif // __MATRIX_HPP__