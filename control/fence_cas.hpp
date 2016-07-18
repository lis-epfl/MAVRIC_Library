/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 ******************************************************************************/

/*******************************************************************************
 * \file fence_cas.cpp
 *
 * \author MAV'RIC Team
 * \author Cyril Stuber
 *
 * \brief 	This module takes care of simulating a fence and avoiding it.
 * 			"cas" stand for Collision Avoiding System.
 *
 ******************************************************************************/

#ifndef FENCE_CAS_H_
#define FENCE_CAS_H_

#include "communication/mavlink_waypoint_handler.hpp"
#include "sensing/position_estimation.hpp"


extern "C"
{
#include "control/stabilisation.h"
#include "sensing/ahrs.h"
}

typedef enum FENCE_INTERP
{
	LINEAR=0,
	COSINE=1,
	SHIFTED_COS=2
} FENCE_INTERP;


class Fence_CAS
{
public:
    /**
     * \brief   Constructor
     *
     * \param   waypoint_handler    For fencepoints
     * \param   position_estimation For position and speed
     * \param   controls            For resetting the roll command
     */
	Fence_CAS(Mavlink_waypoint_handler* waypoint_handler, Position_estimation* postion_estimation,ahrs_t* ahrs );
	~Fence_CAS(void);
    /**
     * \brief   Main update function - compute the repulsion with the fences
     *
     * \return  success
     */
	bool 	update(void);
    /**
     * \brief   Returns the repulsion on the choose axis
     *
     * \param index of the axis [0,1,2] = [X,ROLL,Z]
     *
     * \return  repulsion value
     */
	float 	get_repulsion(int axis);
    /**
     * \brief   clip the repulsion in function of actual velocity norm
     *
     * \param pointer on control_command_t to know the actual velocity norm
     *
     * \return true
     */
	bool 	clip_repulsion(control_command_t command_t);
    /**
     * \brief   Returns the maximal y speed per update
     *
     * \return  maximal angle per update
     *
     */
	float 	get_max_vel_y(void);
    /**
     * \brief   Transform Global frame to body frame coordinates
     *
     * \param Center of new coordinate system
     *
     * \param Heading of new coordinate system
     *
     * \param vector to transform
     *
     */
	void 	gftobftransform(float C[3], float S[3], float rep[3]);
    /**
     * \brief   Choose the interpolation function for the repulsion
     *
     * \param value to interpolate
     *
     * \param type of interpolation [0,1,2] = [linear,cos,cos2]
     *
     * \return  interpolation value
     */
	float 	interpolate(float r, int type);
    /**
     * \brief   Return the angular acceleration in function of the norm of the velocity
     *
     * \param norm of the velocity
     *
     * \return  acceleration value
     */
	float getacc(float normvel);
    /**
     * \brief   Return the maximal possible radius in function of the norm of the velocity
     *
     * \param norm of the velocity
     *
     * \return  radius value
     */
	float getrad(float normvel);


	float	maxsens;	///< Maximal detection distance, 		typically 10
	float	a_max; 		///< Maximal deceleration [m/s^2], 		typically 1
	float	r_pz; 		///< Radius of Protection Zone, 		typically 0.5
	float	comfort; 	///< [0,1] Intensity of the reaction, 	typically 0.5
	float	tahead; 	///< [0,3] Intensity of the reaction, 	typically = 3*comfort
	float	coef_roll; 	///< [0,1] Intensity of the reaction, 	typically 1
	float	maxradius; 	///< [0,100] Maximal radius of curvature, 	typically 5
	float	max_vel_y;  ///< [0,2] Maximal speed in y direction, 	typically 1

private:
    /**
     * \brief Detect the point of intersection of two segment and return the smallest distance between them
     *
     * \param Segment 1 point A (Fence A)
     * \param Segment 1 point B (Fence B)
     *
     * \param Segment 2 point A (Quand center)
     * \param Segment 2 point B	(Quad heading)
     *
     * \param Velocity of the quad
     *
     * \param Interception point on segment 1 (Fence segment)
     * \param Interception point on segment 2 (Quad segment)
     *
     * \param ponter on int, 1/0/-1 = detection of leftside /on the segment / rightside of the segment
     *
     * \return  distance between the segments
     */
	float 	detect_seg(float A[3], float B[3], float C[3], float S[3] , float V[3], float I[3],float J[3],int* outofseg);

	Mavlink_waypoint_handler* 	waypoint_handler;			///< Waypoint handler (extract fencepoints)
	Position_estimation*        pos_est;                    ///< Estimated position and speed (extract the velocity and the position)
	ahrs_t* 					ahrs ;						///< Accelaration
	float 						repulsion[3];				///< Repulsion vector in semi-local frame (only act on ROLL, rep[1])
};

#endif /*FENCE_CAS_H_*/
