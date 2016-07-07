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
 * \file    kalman.hpp
 *
 * \author  MAV'RIC Team
 * \author  Julien Lecoeur
 *
 * \brief   Kalman filter
 * 
 * \details Convention used for dimensions:
 *          - n  state
 *          - p  input
 *          - m  measurement
 *
 ******************************************************************************/


#ifndef KALMAN_HPP__
#define KALMAN_HPP__

#include "matrix.hpp"


namespace kf
{

/**
 * \brief Perform kalman predict step without input
 * 
 * \param x State
 * \param P State covariance
 * \param F Process
 * \param Q Process noise
 * 
 * \tparam n Size of state vector
 * \tparam T Type of data
 */
template<uint32_t n, typename T>
static void predict(Mat<n,1,T>& x,
                    Mat<n,n,T>& P,
                    const Mat<n,n,T>& F,  
                    const Mat<n,n,T>& Q)
{
    // State
    x = F % x;

    // State covariance
    P = (F % P % ~F) + Q;
};


/**
 * \brief Perform kalman predict step with input
 * 
 * \param x State
 * \param P State covariance
 * \param F Process
 * \param Q Process noise
 * \param B Input model
 * \param u Input vector
 * 
 * \tparam n Size of state vector
 * \tparam p Size of input vector
 * \tparam T Type of data
 */
template<uint32_t n, uint32_t p, typename T>
static void predict(Mat<n,1,T>& x,
                    Mat<n,n,T>& P,
                    const Mat<n,n,T>& F,  
                    const Mat<n,n,T>& Q,  
                    const Mat<n,p,T>& B,  
                    const Mat<p,1,T>& u)
{
    // Do prediction without input
    predict(x, P, F, Q);

    // Add effect of input
    x += B % u;
};


/**
 * \brief Perform kalman predict step with input
 * 
 * \param x State
 * \param P State covariance
 * \param F Process
 * \param Q Process noise
 * \param B Input model
 * \param u Input vector
 * 
 * \tparam n Size of state vector
 * \tparam p Size of input vector
 * \tparam T Type of data
 */
template<uint32_t n, uint32_t m, typename T>
static void update(Mat<n,1,T>& x,
                   Mat<n,n,T>& P,
                   Mat<m,1,T>& z,
                   const Mat<m,n,T>& H,  
                   const Mat<m,m,T>& R,  
                   Mat<m,m,T>& S,  
                   Mat<n,m,T>& K,  
                   Mat<m,1,T>& y,
                   const Mat<n,n,T>& I)
{
    // Innovation
    y = z - H % x;

    // Innovation covariance
    S = H % P % ~H + R;

    // Kalman gain
    bool inversible;
    K = P % ~H % S.inv(inversible);

    if(inversible)
    {
        // Update state
        x = x + K % y;

        // Update state covariance
        P = (I - K % H) % P;
    }
};

}


/**
 * \brief Templated linear kalman filter
 * 
 * \tparam n    Size of state vector
 * \tparam p    Size of input vector
 * \tparam m    Size of measure vector
 * \tparam T    Type of data (default: float) 
 */
template<uint32_t n, uint32_t p, uint32_t m, typename T=float>
class Kalman
{
public:

    /**
     * \brief Default constructor
     */
    Kalman(void):
        x_(),
        P_(1.0f, true),
        F_(1.0f, true),
        Q_(0.01f, true),
        H_(),
        R_(0.01f, true),
        B_(),
        S_(),
        I_(1.0f, true),
        K_(),
        y_()
    {}


    /**
     * \brief Constructor
     * 
     * \param x     State vector (initial value)
     * \param P     State covariance (initial value) 
     * \param Q     Process noise covariance
     * \param H     Measurement matrix
     * \param R     Measurement noise
     * \param B     Input matrix (default null)
     */
    Kalman(Mat<n,1> x, Mat<n,n> P, Mat<n,n> F, Mat<n,n> Q, Mat<m,n> H, Mat<m,m> R, Mat<n,p> B = Mat<n,p>()):
        x_(x),
        P_(P),
        F_(F),
        Q_(Q),
        H_(H),
        R_(R),
        B_(B),
        S_(),
        I_(1.0f, true),
        K_(),
        y_()
    {}    


    /**
     * \brief   Get current state
     * 
     * \return  Reference to state (const)
     */
    const Mat<n,1>& x(void) const
    {
        return x_;
    };


    /**
     * \brief   Get current state covariance
     * 
     * \return  Reference to state covariance (const)
     */
    const Mat<n,n>& P(void) const
    {
        return P_;
    };


    /**
     * \brief   Predict next state without input
     */
    void predict(void)
    {
        kf::predict(x_, P_, F_, Q_);
    }


    /**
     * \brief   Get current state with input 
     */
    void predict(Mat<p,1,T> u)
    {
        kf::predict(x_, P_, F_, Q_, B_, u);
    }


    /**
     * \brief Update
     * 
     * \param z     Measurement vector
     */
    void update(Mat<m,1,T> z)
    {
        kf::update(x_, P_, z, H_, R_, S_, K_, y_, I_);
    }


    /**
     * \brief       Update using different measurement matrix and noise 
     * 
     * \details     The measurement vector can be of any size
     * 
     * \param   z   Measurement vector
     * \param   H   Measurement matrix
     * \param   R   Measurement noise
     * 
     * \tparam  mm  Size of measurement vector
     */
    template<uint32_t mm>
    void update(Mat<mm,1,T> z, Mat<mm,n,T> H, Mat<mm,mm,T> R)
    {
        Mat<mm,mm,T> S;
        Mat<n,mm,T> K;
        Mat<mm,1,T> y;
        kf::update(x_, P_, z, H, R, S, K, y, I_);
    }


protected:

    Mat<n,1> x_;    ///< State
    Mat<n,n> P_;    ///< State covariance
    Mat<n,n> F_;    ///< Process
    Mat<n,n> Q_;    ///< Process noise
    Mat<m,n> H_;    ///< Measurement
    Mat<m,m> R_;    ///< Measurement noise
    Mat<n,p> B_;    ///< Input

    Mat<m,m> S_;    ///< Innovation covariance
    const Mat<n,n> I_;    ///< Identity matrix
    Mat<n,m> K_;    ///< Kalman gain
    Mat<m,1> y_;    ///< Innovation
};


#endif /* KALMAN_HPP__ */