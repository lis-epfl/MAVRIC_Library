 /** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 
 
/**
 * \file orca.c
 *
 * This file computes a collision-free trajectory for the ORCA algorithm
 */


#include "orca.h"
#include "neighbor_selection.h"
#include "central_data.h"
#include "maths.h"
#include "print_util.h"
#include "maths.h"

central_data_t *centralData;

float timeHorizon, invTimeHorizon;

int8_t loop_count_orca = 0;
int8_t loop_count_collisions = 0;

float min_coll_dist;

void init_orca(void)
{
	centralData = central_data_get_pointer_to_struct();
	centralData->safe_size = SIZE_VHC_ORCA;
		
	timeHorizon = TIME_HORIZON;
	invTimeHorizon = 1.0f/timeHorizon;

	min_coll_dist = 2.0f * SIZE_VHC_ORCA + 1.0f;
}

void computeNewVelocity(float OptimalVelocity[], float NewVelocity[])
{
	uint8_t ind, i;
	
	plane_t planes[MAX_NUM_NEIGHBORS];
	
	UQuat_t q_neighbor, q_neighbor_bf;
	
	float relativePosition[3], relativeVelocity[3];
	float combinedRadius, distSq, combinedRadiusSq, dotProduct, wLength, wLenghtSq;
	
	float w[3], unitW[3], u[3], neighor_bf[3];
	
	for (i=0;i<3;i++)
	{
		NewVelocity[i] = OptimalVelocity[i];
	}
	
	neighbors_selection_extrapolate_or_delete_position(centralData->listNeighbors, &(centralData->number_of_neighbors));
	
	// Create agent ORCA planes
	for (ind=0; ind<centralData->number_of_neighbors; ind++)
	{
		// Linear extrapolation of the position of the neighbor between two received messages
		for (i=0;i<3;i++)
		{
			relativePosition[i] = centralData->listNeighbors[ind].extrapolatedPosition[i] - centralData->position_estimator.localPosition.pos[i];
			relativeVelocity[i] = centralData->position_estimator.vel[i] - centralData->listNeighbors[ind].velocity[i];
		}
		
		q_neighbor.s = 0.0f;
		q_neighbor.v[0] = relativeVelocity[0];
		q_neighbor.v[1] = relativeVelocity[1];
		q_neighbor.v[2] = relativeVelocity[2];
		q_neighbor_bf = quat_global_to_local(centralData->imu1.attitude.qe,q_neighbor);
		
		neighor_bf[0] = q_neighbor_bf.v[0];
		neighor_bf[1] = q_neighbor_bf.v[1];
		neighor_bf[2] = q_neighbor_bf.v[2];
		
		for (i=0;i<3;i++)
		{
			relativeVelocity[i] = neighor_bf[i];
		}
		
		q_neighbor.s = 0.0f;
		q_neighbor.v[0] = relativePosition[0];
		q_neighbor.v[1] = relativePosition[1];
		q_neighbor.v[2] = relativePosition[2];
		q_neighbor_bf = quat_global_to_local(centralData->imu1.attitude.qe,q_neighbor);
		
		neighor_bf[0] = q_neighbor_bf.v[0];
		neighor_bf[1] = q_neighbor_bf.v[1];
		neighor_bf[2] = q_neighbor_bf.v[2];
		
		for (i=0;i<3;i++)
		{
			relativePosition[i] = neighor_bf[i];
		}
		
		distSq = vector_norm_sqr(relativePosition);
		combinedRadius = centralData->safe_size + centralData->listNeighbors[ind].size;
		combinedRadiusSq = SQR(combinedRadius);
		
		
		if (distSq > combinedRadiusSq)
		{
			// No collisions
			for (i=0;i<3;i++)
			{
				w[i] = relativeVelocity[i] - invTimeHorizon * relativePosition[i];
			}
			wLenghtSq = vector_norm_sqr(w);
			
			dotProduct = scalar_product(w,relativePosition);
			
			if ((dotProduct < 0.0f)&&(SQR(dotProduct) > (combinedRadiusSq * wLenghtSq)))
			{
				// Project on cut-off circle
				wLength = fast_sqrt(wLenghtSq);
				for (i=0;i<3;i++)
				{
					unitW[i] = w[i] / wLength;
					planes[ind].normal[i] = unitW[i];
					u[i] = (combinedRadius * invTimeHorizon - wLength) * unitW[i];
				}
			}
			else
			{
				// Project on cone
				float a = distSq;
				float b = scalar_product(relativePosition,relativeVelocity);
				float crossProduct[3];
				CROSS(relativePosition,relativeVelocity,crossProduct);
				float c = vector_norm_sqr(relativeVelocity) - vector_norm_sqr(crossProduct) / (distSq - combinedRadiusSq);
				float t = (b + fast_sqrt(SQR(b) - a*c)) / a;
				for (i=0;i<3;i++)
				{
					w[i] = relativeVelocity[i] - t * relativePosition[i];
				}
				wLength = vector_norm(w);
				for (i=0;i<3;i++)
				{
					unitW[i] = w[i] / wLength;
					planes[ind].normal[i] = unitW[i];
					u[i] = (combinedRadius * t - wLength) * unitW[i];
				}
			}
		}
		else
		{
			// Collisions
			min_coll_dist = f_min(min_coll_dist,sqrt(distSq));
			
			loop_count_collisions = loop_count_collisions++ % 100;
			if (loop_count_collisions == 0)
			{
				dbg_print("Collision! ");
				dbg_print("Distance with neighbor ");
				dbg_print_num(ind,10);
				dbg_print("(x100):");
				dbg_print_num(sqrt(distSq)*100.0f,10);
				dbg_print(", min dist:");
				dbg_print_num(min_coll_dist*100.0f,10);
				dbg_print("\n");
			}
			
			float invTimeStep = 1.0f / ORCA_TIME_STEP_MILLIS; //PROBLEM wrong time step
			
			for (i=0;i<3;i++)
			{
				w[i] = relativeVelocity[i] - invTimeStep * relativePosition[i];
			}
			
			wLength = vector_norm(w);
			
			for (i=0;i<3;i++)
			{
				unitW[i] = w[i] / wLength;
				planes[ind].normal[i] = unitW[i];
				u[i] = (combinedRadius * invTimeStep - wLength) * unitW[i];
			}
		}
		
		for (i=0;i<3;i++)
		{
			planes[ind].point[i] = centralData->position_estimator.vel_bf[i] + 0.5f * u[i];
		}
		
	}
	
	float planeFail = linearProgram3(planes,centralData->number_of_neighbors, OptimalVelocity, MAXSPEED, NewVelocity, false);
	
	if (planeFail < centralData->number_of_neighbors)
	{
		linearProgram4(planes,centralData->number_of_neighbors,planeFail,MAXSPEED,NewVelocity);
	}
	
	loop_count_orca = loop_count_orca++ % 100;
	float orca_diff[3];
	
	for (i=0;i<3;i++)
	{
		orca_diff[i] = OptimalVelocity[i] - NewVelocity[i];
	}
	
	if (loop_count_orca == 0)
	{
		dbg_print("Orca diffvel:");
		dbg_print_vector(orca_diff,2);
		dbg_print(", Optimal:");
		dbg_print_vector(OptimalVelocity,2);
		dbg_print(", New:");
		dbg_print_vector(NewVelocity,2);
		dbg_print("\n");
	/*}
	else
	{
		if (vector_norm_sqr(orca_diff)>0.2)
		{
			dbg_print("Orca diffvel:");
			dbg_print_vector(orca_diff,2);
			dbg_print(", Optimal:");
			dbg_print_vector(OptimalVelocity,2);
			dbg_print(", New:");
			dbg_print_vector(NewVelocity,2);
			dbg_print("\n");
		}*/
	}
}

bool linearProgram1(plane_t planes[], uint8_t index, line_t line, float maxSpeed, float OptimalVelocity[], float NewVelocity[], bool directionOpt)
{
	uint8_t i;
	
	float dotProduct = scalar_product(line.point,line.direction);
	float discriminant = SQR(dotProduct) + SQR(maxSpeed) - vector_norm_sqr(line.point);
	
	if (discriminant < 0.0f)
	{
		// Max speed sphere fully invalidates line
		return false;
	}
	
	float sqrtDiscriminant = fast_sqrt(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;
	
	uint8_t index2;
	for (index2=0;index2<index;index2++)
	{
		float diffPoints[3];
		
		for (i=0;i<3;i++)
		{
			diffPoints[i] = planes[index2].point[i] - line.point[i];
		}
		
		float numerator = scalar_product(diffPoints, planes[index2].normal);
		float denominator = scalar_product(line.direction, planes[index2].normal);
		
		if (SQR(denominator) <= RVO_EPSILON)
		{
			// Lines line is (almost) parallel to plane i
			if (numerator > 0.0f)
			{
				return false;
			}
			else
			{
				continue;
			}
		}
		
		float t = numerator / denominator;
		
		if (denominator >= 0.0f)
		{
			// Plane i bounds line on the left
			tLeft = f_max(tLeft, t);
		}
		else
		{
			// Plane i bounds line on the right
			tRight = f_min(tRight, t);
		}
		
		if (tLeft > tRight)
		{
			return false;
		}
	}
	
	if (directionOpt)
	{
		// Optimize direction
		if (scalar_product(OptimalVelocity, line.direction) > 0.0f) 
		{
			// Take right extreme
			for (i=0;i<3;i++)
			{
				NewVelocity[i] = line.point[i] + tRight * line.direction[i];
			}
		}
		else 
		{
			// Take left extreme
			for (i=0;i<3;i++)
			{
				NewVelocity[i] = line.point[i] + tLeft * line.direction[i];
			}
		}
	}
	else
	{
		// Optimize closest point
		float diffVelPoint[3];
		for (i=0;i<3;i++)
		{
			diffVelPoint[i] = OptimalVelocity[i] - line.point[i];
		}
		
		float t = scalar_product(line.direction, diffVelPoint);

		if (t < tLeft)
		{
			for (i=0;i<3;i++)
			{
				NewVelocity[i] = line.point[i] + tLeft * line.direction[i];
			}
		}
		else
		{
			if (t > tRight)
			{
				for (i=0;i<3;i++)
				{
					NewVelocity[i] = line.point[i] + tRight * line.direction[i];
				}
			}
			else
			{
				for (i=0;i<3;i++)
				{
					NewVelocity[i] = line.point[i] + t * line.direction[i];
				}
			}
		}
	}
	return true;
}

bool linearProgram2(plane_t planes[], uint8_t ind, float maxSpeed, float OptimalVelocity[], float NewVelocity[], bool directionOpt)
{
	uint8_t i;
	uint8_t index;
	
	float planeDist = scalar_product(planes[ind].point,planes[ind].normal);
	float planeDistSq = SQR(planeDist);
	float radiusSq = SQR(maxSpeed);
	
	if (planeDistSq > radiusSq)
	{
		// Max speed sphere fully invalidates plane planeNo
		return false;
	}
	
	float planeRadiusSq = radiusSq - planeDistSq;
	
	float planeCenter[3];
	for(i=0;i<3;i++)
	{
		planeCenter[i] = planeDist * planes[ind].normal[i];
	}
	
	if (directionOpt)
	{
		// Project direction optVelocity on plane ind
		float planeOptVelocity[3];
		float scalarProduct = scalar_product(OptimalVelocity,planes[ind].normal);
		
		for(i=0;i<3;i++)
		{
			planeOptVelocity[i] = OptimalVelocity[i] - scalarProduct * planes[ind].normal[i];
		}
		
		float planeOptVelocityLengthSq = vector_norm_sqr(planeOptVelocity);
		
		if (planeOptVelocityLengthSq <= RVO_EPSILON)
		{
			for(i=0;i<3;i++)
			{
				NewVelocity[i] = planeCenter[i];
			}
		}
		else
		{
			float sqrtPlane = fast_sqrt(planeRadiusSq / planeOptVelocityLengthSq);
			
			for(i=0;i<3;i++)
			{
				NewVelocity[i] = planeCenter[i] + sqrtPlane * planeOptVelocity[i];
			}
		}
	}
	else
	{
		// Project point optVelocity on plane ind
		float diffPtsVel[3];
		
		for(i=0;i<3;i++)
		{
			diffPtsVel[i] = planes[ind].point[i]-OptimalVelocity[i];
		}
		
		float scalarProduct = scalar_product(diffPtsVel,planes[ind].normal);
		
		for(i=0;i<3;i++)
		{
			NewVelocity[i] = OptimalVelocity[i] + scalarProduct * planes[ind].normal[i];
		}
		// If outside planeCircle, project on planeCircle
		if (vector_norm_sqr(NewVelocity) > radiusSq)
		{
			float planeResult[3];
			
			for(i=0;i<3;i++)
			{
				planeResult[i] = NewVelocity[i] - planeCenter[i];
			}
			
			float planeResultLengthSq = vector_norm_sqr(planeResult);
			
			float planeSqrt = fast_sqrt(planeRadiusSq / planeResultLengthSq);
			
			for(i=0;i<3;i++)
			{
				NewVelocity[i] = planeCenter[i] + planeSqrt * planeResult[i];
			}
		}
	}
	
	for (index=0;index<ind;index++)
	{
		float diffPtsNewVel[3];
		for (i=0;i<3;i++)
		{
			diffPtsNewVel[i] = planes[index].point[i] - NewVelocity[i];
		}
		if (scalar_product(planes[index].normal,diffPtsNewVel)>0.0f)
		{
			/* Result does not satisfy constraint index. Compute new optimal result. */
			/* Compute intersection line of plane index and plane ind. */
			float crossProduct[3];
			CROSS(planes[index].normal,planes[ind].normal,crossProduct);
			
			if (vector_norm_sqr(crossProduct) <= RVO_EPSILON)
			{
				/* Planes ind and index are (almost) parallel, and plane index fully invalidates plane ind. */
				return false;
			}
			
			line_t line;
			float normCrossProduct = vector_norm(crossProduct);
			for (i=0;i<3;i++)
			{
				line.direction[i] = crossProduct[i] / normCrossProduct;
			}
			float lineNormal[3];
			CROSS(line.direction,planes[ind].normal,lineNormal);
			
			float diffPoints[3];
			for (i=0;i<3;i++)
			{
				diffPoints[i] = planes[index].point[i] - planes[ind].point[i];
			}
			float scalarProductPointsNormal = scalar_product(diffPoints,planes[index].normal);
			float scalarProductNormals = scalar_product(lineNormal,planes[index].normal);
			for(i=0;i<3;i++)
			{
				line.point[i] = planes[ind].point[i] + (scalarProductPointsNormal / scalarProductNormals) * lineNormal[i];
			}
			
			if (!(linearProgram1(planes,index,line,maxSpeed,OptimalVelocity,NewVelocity,directionOpt)))
			{
				return false;
			}
		}
	}
	return true;
}


float linearProgram3(plane_t planes[], uint8_t planeSize, float OptimalVelocity[], float maxSpeed, float NewVelocity[], bool directionOpt)
{
	uint8_t i;
	
	if (directionOpt)
	{
		/* Optimize direction. Note that the optimization velocity is of unit length in this case. */
		float normOptimalVelocity = vector_norm(OptimalVelocity);
		for(i=0;i<3;i++)
		{
			NewVelocity[i] = OptimalVelocity[i]/normOptimalVelocity * maxSpeed;
		}
	}
	else
	{
		if (vector_norm_sqr(OptimalVelocity) > SQR(maxSpeed))
		{
			/* Optimize closest point and outside circle. */
			float normOptimalVelocity = vector_norm(OptimalVelocity);
			for(i=0;i<3;i++)
			{
				NewVelocity[i] = OptimalVelocity[i]/normOptimalVelocity * maxSpeed;
			}
		}
		else
		{
			for(i=0;i<3;i++)
			{
				NewVelocity[i] = OptimalVelocity[i];
			}
		}
	}
	
	uint8_t ind;
	
	for (ind=0;ind<planeSize;ind++)
	{
		float diffPointVel[3];
		for (i=0;i<3;i++)
		{
			diffPointVel[i] = planes[ind].point[i] - NewVelocity[i];
		}
		if (scalar_product(planes[ind].normal, diffPointVel ) > 0.0f)
		{
			/* Result does not satisfy constraint ind. Compute new optimal result. */
			float tempResult[3];
			for (i=0;i<3;i++)
			{
				tempResult[i] = NewVelocity[i];
			}
			if (!(linearProgram2(planes,ind,maxSpeed,OptimalVelocity,NewVelocity,directionOpt)))
			{
				for (i=0;i<3;i++)
				{
					NewVelocity[i] = tempResult[i];
				}
				
				return ind;
			}
		}
	}
	return planeSize;
}

void linearProgram4(plane_t planes[], uint8_t planeSize, uint8_t ind, float maxSpeed, float NewVelocity[])
{
	uint8_t i;
	
	uint8_t index,index2;
	
	plane_t projPlanes[MAX_NUM_NEIGHBORS];
	
	float distance = 0.0f;
	
	for (index = ind;index < planeSize;index++)
	{
		float diffPointVel[3];
		for (i=0;i<3;i++)
		{
			diffPointVel[i] = planes[index].point[i] - NewVelocity[i];
		}
		if (scalar_product(planes[index].normal,diffPointVel)>distance)
		{
			/* Result does not satisfy constraint of plane i. */
			
			for (index2 = 0;index2<index;index2++)
			{
				plane_t plane;
				float crossProduct[3];
				CROSS(planes[index2].normal, planes[index].normal, crossProduct);
				
				if (vector_norm_sqr(crossProduct)<=RVO_EPSILON)
				{
					/* Plane index and plane index2 are (almost) parallel. */
					if (scalar_product(planes[index].normal, planes[index2].normal) > 0.0f)
					{
						/* Plane index and plane index2 point in the same direction. */
						continue;
					}
					else
					{
						/* Plane index and plane index2 point in opposite direction. */
						for (i=0;i<3;i++)
						{
							plane.point[i] = 0.5f * (planes[index].point[i] + planes[index2].point[i]);
						}
					}
				}
				else
				{
					float lineNormal[3];
					CROSS(crossProduct,planes[index].normal,lineNormal);
					
					float diffPoints[3];
					for (i=0;i<3;i++)
					{
						diffPoints[i] = planes[index2].point[i] - planes[index].point[i];
					}
					
					float scalarProdPtsNormal = scalar_product(diffPoints, planes[index2].normal);
					float scalarProdNormals = scalar_product(lineNormal, planes[index2].normal);
					for (i=0;i<3;i++)
					{
						plane.point[i] = planes[index].point[i] + (scalarProdPtsNormal / scalarProdNormals) * lineNormal[i];
					}
				}
				
				for (i=0;i<3;i++)
				{
					plane.normal[i] = planes[index2].normal[i] - planes[index].normal[i];
				}
				float normNormal = vector_norm(plane.normal);
				for(i=0;i<3;i++)
				{
					plane.normal[i] = plane.normal[i] / normNormal;
				}
				
				projPlanes[index2] = plane;
			}
			
			float tempResult[3];
			for (i=0;i<3;i++)
			{
				tempResult[i] = NewVelocity[i];
			}
			if (linearProgram3(projPlanes,index2,planes[index].normal,maxSpeed,NewVelocity,true)<index2)
			{
				for (i=0;i<3;i++)
				{
					NewVelocity[i] = tempResult[i];
				}
			}
			float diffPointVel[3];
			for (i=0;i<3;i++)
			{
				diffPointVel[i] = planes[index].point[i] - NewVelocity[i];
			}
			distance = scalar_product(planes[index].normal,diffPointVel);
		}
	}
}