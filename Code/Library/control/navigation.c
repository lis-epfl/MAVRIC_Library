/*
 * navigation.c
 *
 * Created: 15.05.2013 10:57:21
 *  Author: Philippe
 */ 

#include "boardsupport.h"
#include "time_keeper.h"
#include "control.h"
#include "stabilisation.h"
#include "waypoint_navigation.h"
#include "navigation.h"


#define LIMIT_ZERO_SPEED		0.1 // 10cm/s
#define LIMIT_SPEED_NAV_MODE	1 // m/s
#define MAX_SPEED				4 // m/s , max speed for the global speed
#define WAYPOINT_RADIUS			3 // m / tolerance on the waypoint's position 
#define SLOWDOWN_RADIUS			8 // m / distance from which the quad starts slowing down
#define INVERSE_SLOW_RADIUS		1/SLOWDOWN_RADIUS 

#define PI						3.1415
#define PI_HALF					3.1415/2.

#define MAX_PHI					10.*PI/180.
#define MAX_THETA				10.*PI/180.
#define MAX_THRUST				1.2
#define YAW_TOLERANCE			0.1 //rad
#define MAX_ACC					0.587 // 1/(g*sin(MAX_ANGLE)) with MAX_ANGLE=10degree ,used to calculate radius of circle for circular fly
#define DELTA_T					0.02 // mean delta_t between each loop


#define NAV_MODE_NULL -1
//Stays at the same position
#define NAV_MODE_STATIC 0		
//Stops, turns and goes straight to the desired position	
#define NAV_MODE_POINT_N_GO 1	
//Goes direct to the desired position without slowing
#define NAV_MODE_DIRECT 2		

#define TD_X		0
#define TI_X		100000
#define KP_X		1
#define TD_Y		0
#define TI_Y		100000
#define KP_Y		1
#define TD_Z		0
#define TI_Z		100000
//Kp_thrust must be bigger than other Kp
#define KP_Z		1



board_hardware_t *board;
SpeedContr_Data_t setpoint;  //pid parameters
Navigation_Data_t nav_data;  //goal coord and on/off
int8_t navigation_mode=NAV_MODE_NULL;
int8_t current_waypoint=-1;





void init_navigation()
{
	board = get_board_hardware();
	// X speed PID init
	(setpoint.pid_data[PID_X]).p_gain=KP_X;
	(setpoint.pid_data[PID_X]).last_update=get_time_ticks();
	(setpoint.pid_data[PID_X]).clip_min= -MAX_THETA;
	(setpoint.pid_data[PID_X]).clip_max= MAX_THETA;
	initDiff(&(setpoint.pid_data[PID_X].differentiator), TD_X, 0.8, 0.5);	//Td,LPF,clip
	initInt(&(setpoint.pid_data[PID_X].integrator),1, 1/TI_X, 0.1);			//pregain,postgain,clip
	// Y speed PID init	
	(setpoint.pid_data[PID_Y]).p_gain=KP_Y;
	(setpoint.pid_data[PID_Y]).last_update=get_time_ticks();
	(setpoint.pid_data[PID_Y]).clip_min= -MAX_PHI;
	(setpoint.pid_data[PID_Y]).clip_max= MAX_PHI;	
	initDiff(&(setpoint.pid_data[PID_Y].differentiator), TD_Y, 0.8, 0.5);	
	initInt(&(setpoint.pid_data[PID_Y].integrator),1, 1/TI_Y, 0.1);
	// Z speed PID init
	(setpoint.pid_data[PID_Z]).p_gain=KP_Z;
	(setpoint.pid_data[PID_Z]).last_update=get_time_ticks();
	(setpoint.pid_data[PID_Z]).clip_min= -MAX_THRUST;
	(setpoint.pid_data[PID_Z]).clip_max= MAX_THRUST;
	initDiff(&(setpoint.pid_data[PID_Z].differentiator), TD_Z, 0.8, 0.5);
	initInt(&(setpoint.pid_data[PID_Z].integrator),1, 1/TI_Z, 0.1);
}

void nav_function(int nav_type, int current_waypoint)
{
	static int8_t nav_phase=0;
	int i=0;
	float x_vec[3],dist_vec[3],yaw_waypoint=0,yaw_actual=0,distance_waypoint=0,inverse_dist_norm=0;

	if (nav_type==NAV_MODE_DIRECT && nav_phase==0)
		nav_phase=2; //do not do steps 0 and 1

	switch (nav_phase)
	{
	//set speed to 0
	case 0:
	{
		setpoint.x_speed=0;
		setpoint.y_speed=0;
		setpoint.z_speed=0;
		if (board->estimation.state[0][1]<LIMIT_ZERO_SPEED && board->estimation.state[1][1]<LIMIT_ZERO_SPEED && board->estimation.state[2][1]<LIMIT_ZERO_SPEED)
		{
			yaw_waypoint=get_angle_with_XNED(board->waypoint_list[current_waypoint].x-board->estimation.state[0][0],board->waypoint_list[current_waypoint].y-board->estimation.state[1][0],board->waypoint_list[current_waypoint].z-board->estimation.state[2][0]); //Set yaw angle setpoint
			setpoint.yaw=yaw_waypoint;
			nav_phase=1;//goes to next phase
		}		
		break;
	}
	//wait until yaw align with the goal
	case 1:
	{
		x_vec[0]=1;x_vec[1]=0;x_vec[2]=0;
		quat_rot(&(board->imu1.attitude.qe),x_vec);
		yaw_actual=get_angle_with_XNED(x_vec[0],x_vec[1],x_vec[2]);
		if (yaw_actual<(yaw_waypoint+YAW_TOLERANCE) && yaw_actual>(yaw_waypoint-YAW_TOLERANCE)) //if yaw is set
			nav_phase=2;
		break;
	}
	//move in direction of the goal
	case 2: //Start here if NAV_DIRECT
	{
		dist_vec[0] = board->waypoint_list[current_waypoint].x-board->estimation.state[0][0];
		dist_vec[1] = board->waypoint_list[current_waypoint].y-board->estimation.state[1][0];
		dist_vec[2] = -board->waypoint_list[current_waypoint].z-board->estimation.state[2][0];	
		if (get_angle(dist_vec[0],dist_vec[1],dist_vec[2],board->estimation.state[0][1],board->estimation.state[1][1],board->estimation.state[2][1])<=PI_HALF) //if the angle between speed and goal is smaller than pi/2 regular nav, else circular_nav
		{
			distance_waypoint = dist_vec[0]*dist_vec[0]+dist_vec[1]*dist_vec[1]+dist_vec[2]*dist_vec[2]; //square of distance
			inverse_dist_norm = Q_rsqrt(distance_waypoint);
			if (distance_waypoint>(SLOWDOWN_RADIUS*SLOWDOWN_RADIUS)) //If we're far, Max_speed in direction of the waypoint
			{
				setpoint.x_speed=dist_vec[0]*inverse_dist_norm*MAX_SPEED;
				setpoint.y_speed=dist_vec[1]*inverse_dist_norm*MAX_SPEED;
				setpoint.z_speed=-(dist_vec[2]*inverse_dist_norm*MAX_SPEED); // if the quad needs to go up, the command will be negative, thus there is a minus
				setpoint.yaw=get_angle_with_XNED(board->estimation.state[0][1],board->estimation.state[1][1],board->estimation.state[2][1]); //Set yaw angle setpoint align with the speed
			}
			else if (distance_waypoint<=(SLOWDOWN_RADIUS*SLOWDOWN_RADIUS))
			{
				if (board->waypoint_list[current_waypoint].autocontinue==0)	//then start slowing down in phase 3
					nav_phase=3;
				else if (distance_waypoint>=(WAYPOINT_RADIUS*WAYPOINT_RADIUS) && board->waypoint_list[current_waypoint].autocontinue==1)//else keep going at the same speed until waypoint radius
				{
					setpoint.x_speed=dist_vec[0]*inverse_dist_norm*MAX_SPEED;
					setpoint.y_speed=dist_vec[1]*inverse_dist_norm*MAX_SPEED;
					setpoint.z_speed=-(dist_vec[2]*inverse_dist_norm*MAX_SPEED); // if the quad needs to go up, the command will be negative, thus there is a minus
					setpoint.yaw=get_angle_with_XNED(board->estimation.state[0][1],board->estimation.state[1][1],board->estimation.state[2][1]); //Set yaw angle setpoint align with the speed
				}
				else if (distance_waypoint<(WAYPOINT_RADIUS*WAYPOINT_RADIUS) && board->waypoint_list[current_waypoint].autocontinue==1)
				{
					board->waypoint_list[current_waypoint].current=0;
					current_waypoint++;	
					
					if (current_waypoint>=board->number_of_waypoints) //test for next waypoint
						current_waypoint=0;			

					board->waypoint_list[current_waypoint].current=1;
					navigation_mode=NAV_MODE_DIRECT;
					nav_phase=0;
				}
			}
		}
		else
			circular_fly();	 //If the waypoint is behind the quadrotor, this function computes the speed's setpoints
		break;
	}
	//slow near the goal if autocontinuous = 0
	case 3:
	{
		dist_vec[0] = board->waypoint_list[current_waypoint].x-board->estimation.state[0][0];
		dist_vec[1] = board->waypoint_list[current_waypoint].y-board->estimation.state[1][0];
		dist_vec[2] = -board->waypoint_list[current_waypoint].z-board->estimation.state[2][0];
		distance_waypoint = dist_vec[0]*dist_vec[0]+dist_vec[1]*dist_vec[1]+dist_vec[2]*dist_vec[2]; //square of distance
		if (distance_waypoint>=(WAYPOINT_RADIUS*WAYPOINT_RADIUS))
		{
			inverse_dist_norm = Q_rsqrt(distance_waypoint);
			setpoint.x_speed=dist_vec[0]*inverse_dist_norm*MAX_SPEED*(dist_vec[0]*INVERSE_SLOW_RADIUS); //same as before but with the speed correction
			setpoint.y_speed=dist_vec[1]*inverse_dist_norm*MAX_SPEED*(dist_vec[1]*INVERSE_SLOW_RADIUS);
			setpoint.z_speed=-(dist_vec[2]*inverse_dist_norm*MAX_SPEED*(dist_vec[2]*INVERSE_SLOW_RADIUS)); // if the quad needs to go up, the command will be negative, thus there is a minus
			setpoint.yaw=get_angle_with_XNED(board->waypoint_list[current_waypoint].x-board->estimation.state[0][0],board->waypoint_list[current_waypoint].y-board->estimation.state[1][0],-(board->waypoint_list[current_waypoint].z+board->estimation.state[2][0])); //Set yaw angle setpoint			
		}
		else if (distance_waypoint<(WAYPOINT_RADIUS*WAYPOINT_RADIUS))
		{
			setpoint.x_speed=0;
			setpoint.y_speed=0;
			setpoint.z_speed=0;		
			nav_phase=0;
			board->waypoint_list[current_waypoint].current=0; 		
		}
		break;
	}
	default:
		break;
	}	
}



void run_navigation() //navigation type
{
	float global_speed_square; //square of the global speed
	int i=0;
	
	setpoint.x_speed=0; // if no waypoint, no speed
	setpoint.y_speed=0;
	setpoint.z_speed=0;
		
	for (i=0;i<board->number_of_waypoints;i++)
		if (board->waypoint_list[i].current==1) // if there is a waypoint, adapt speed
		{
			global_speed_square=board->estimation.state[0][1]*board->estimation.state[0][1]+board->estimation.state[1][1]*board->estimation.state[1][1]+board->estimation.state[2][1]*board->estimation.state[2][1];
			//depending on speed
			if ((navigation_mode==NAV_MODE_NULL)&&(global_speed_square<=(LIMIT_SPEED_NAV_MODE*LIMIT_SPEED_NAV_MODE))||(navigation_mode==NAV_MODE_POINT_N_GO))
			{
				navigation_mode=NAV_MODE_POINT_N_GO;
				nav_function(NAV_MODE_POINT_N_GO,i);
			}	
			else if ((navigation_mode==NAV_MODE_NULL)&&(global_speed_square>(LIMIT_SPEED_NAV_MODE*LIMIT_SPEED_NAV_MODE))||(navigation_mode==NAV_MODE_DIRECT))
			{	
				navigation_mode=NAV_MODE_DIRECT;
				nav_function(NAV_MODE_DIRECT,i);
			}
			else if (navigation_mode==NAV_MODE_STATIC)
			{
				setpoint.x_speed=0;
				setpoint.y_speed=0;
				setpoint.z_speed=0;
			}			
		}	
	speed_control_pid();
}



void speed_control_pid() // Take speed in NED and sends speed in quad ref frame to PID
{
	float x_vec[3],y_vec[3],z_vec[3];
	float x_speed_setpoint,y_speed_setpoint,z_speed_setpoint;
	float speed_norm_square,cos_psi;
	x_vec[0]=1;x_vec[1]=0;x_vec[2]=0; //x vector in NED
	y_vec[0]=0;y_vec[1]=1;y_vec[2]=0; //y vector in NED
	z_vec[0]=0;z_vec[1]=0;z_vec[2]=1; //z vector in NED
	quat_rot(&(board->imu1.attitude.qe),x_vec); //x is now the x of the quad ref frame
	quat_rot(&(board->imu1.attitude.qe),y_vec); //y is now the y of the quad ref frame
	quat_rot(&(board->imu1.attitude.qe),z_vec); //z is now the z of the quad ref frame
	x_speed_setpoint=setpoint.x_speed*x_vec[0]+setpoint.y_speed*x_vec[1]+setpoint.z_speed*x_vec[2]; //projection of the wanted speed in the quad ref with scalar product
	y_speed_setpoint=setpoint.x_speed*y_vec[0]+setpoint.y_speed*y_vec[1]+setpoint.z_speed*y_vec[2]; //projection of the wanted speed in the quad ref with scalar product
	z_speed_setpoint=setpoint.x_speed*z_vec[0]+setpoint.y_speed*z_vec[1]+setpoint.z_speed*z_vec[2]; //projection of the wanted speed in the quad ref with scalar product
	board->controls.rpy[ROLL] = pid_update(&(setpoint.pid_data[PID_X]),board->estimation.state[0][1],x_speed_setpoint); //send calculated x speed to the PID
	board->controls.rpy[PITCH] = pid_update(&(setpoint.pid_data[PID_Y]),board->estimation.state[1][1],y_speed_setpoint);	
	board->controls.thrust = pid_update(&(setpoint.pid_data[PID_Z]),board->estimation.state[2][1],z_speed_setpoint);	
	board->controls.rpy[YAW] = setpoint.yaw;
}



float acos_func(float x) //Use a mix of Taylor approx and Lagrangian interpolation - max error : 3.4 degree or 0.06 rad
{	// x between -1 and +1
	if (x<-0.8 && x>=-1) //lagrange interpolation polynom (-1 -0.95 -0.9 -0.8)
		return (((0.3818*x+1)*1.1416*x+1)*3.4995*x+1)*(-104.4115);
		
	else if (x>=-0.8 && x<=0.8) //Taylor approx around 0
		return PI_HALF-x-0.16666*x*x*x;
		
	else if (x>0.8 && x<=1) //lagrange interpolation polynom (0.8 0.9 0.95 1)
		return (((-0.3818*x+1)*-1.1416*x+1)*-3.4995*x+1)*(107.5531);	
			
	else
		return 404; //error
}



float Q_rsqrt( float number ) // Calculate the inverse square root very precisely and fast, with origin comments, see "Fast Inverse Square Root" on Wikipedia
{
	long i;
	float x2, y;
	const float threehalfs = 1.5;
	
	x2 = number * 0.5;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
	
	return y;
}



float get_angle(float v1_x,float v1_y,float v1_z,float v2_x,float v2_y,float v2_z) //gives the angle between 0 and pi, always positive
{
	float scalar_prod,norm_v1,norm_v2,cos_angle;
	scalar_prod=v1_x*v2_x+v1_y*v2_y+v1_z*v2_z;
	norm_v1=v1_x*v1_x+v1_y*v1_y+v1_z*v1_z;
	norm_v2=v2_x*v2_x+v2_y*v2_y+v2_z*v2_z;	
	cos_angle=scalar_prod*Q_rsqrt(norm_v1)*Q_rsqrt(norm_v2);
	return acos_func(cos_angle);
}



float get_angle_with_XNED(float v1_x,float v1_y,float v1_z) //gives the angle between a vector and the X_NED vector (from 0 to 2pi, positive in direction of y)
{
	float norm_v1,cos_angle;
	norm_v1=v1_x*v1_x+v1_y*v1_y+v1_z*v1_z;
	cos_angle=v1_x*Q_rsqrt(norm_v1);
	if (v1_y>=0)
		return acos_func(cos_angle);
	else 
		return (PI+PI-acos_func(cos_angle));
}



void circular_fly()
{
	float dist_vec[3],normalized_speed[3],delta_speed[3];
	float scalar,radius,speed_norm_square,inv_speed_norm,delta_speed_norm;
	
	dist_vec[0] = board->waypoint_list[current_waypoint].x-board->estimation.state[0][0];
	dist_vec[1] = board->waypoint_list[current_waypoint].y-board->estimation.state[1][0];
	dist_vec[2] = -board->waypoint_list[current_waypoint].z-board->estimation.state[2][0];
	
	
	speed_norm_square=board->estimation.state[0][1]*board->estimation.state[0][1]+board->estimation.state[1][1]*board->estimation.state[1][1]+board->estimation.state[2][1]*board->estimation.state[2][1];
	radius=speed_norm_square*MAX_ACC;
	
	scalar=-board->estimation.state[1][1]*dist_vec[0]+board->estimation.state[0][1]*dist_vec[1];
	
	inv_speed_norm=Q_rsqrt(speed_norm_square);
	normalized_speed[0]=board->estimation.state[0][1]*inv_speed_norm;
	normalized_speed[1]=board->estimation.state[1][1]*inv_speed_norm;
	normalized_speed[2]=board->estimation.state[2][1]*inv_speed_norm;
	
	delta_speed_norm=speed_norm_square*DELTA_T/radius;
	
	delta_speed[0]=-normalized_speed[1]*delta_speed_norm; //calculate the delta_v (perpendicular to v) that'll be add to the actual speed vector (v), to turn the quadrotor
	delta_speed[1]=normalized_speed[0]*delta_speed_norm;
	delta_speed[2]=normalized_speed[2]*delta_speed_norm;

		
	
	if (scalar>=0)//should turn left or right ?
	{//Turn right keeping same speed
		setpoint.x_speed=board->estimation.state[0][1]+delta_speed[0];
		setpoint.y_speed=board->estimation.state[1][1]+delta_speed[1];
		
		//positive setpoint.z_speed for going up, negative for going down
		if (MAX_SPEED-setpoint.x_speed-setpoint.y_speed>0 && dist_vec[2]<0) //needs to go up
			setpoint.z_speed=MAX_SPEED-setpoint.x_speed-setpoint.y_speed; //total speed below Max_speed but easy to calculate, might be changed
		else if (MAX_SPEED-setpoint.x_speed-setpoint.y_speed>0 && dist_vec[2]>0) //needs to go down
			setpoint.z_speed=-(MAX_SPEED-setpoint.x_speed-setpoint.y_speed); 
		else 
			setpoint.z_speed=0;
	}
	else
	{//Turn left
		setpoint.x_speed=board->estimation.state[0][1]-delta_speed[0];
		setpoint.y_speed=board->estimation.state[1][1]-delta_speed[1];
		
		//positive setpoint.z_speed for going up, negative for going down
		if (MAX_SPEED-setpoint.x_speed-setpoint.y_speed>0 && dist_vec[2]<0) //needs to go up
		setpoint.z_speed=MAX_SPEED-setpoint.x_speed-setpoint.y_speed; //total speed below Max_speed but easy to calculate, might be changed
		else if (MAX_SPEED-setpoint.x_speed-setpoint.y_speed>0 && dist_vec[2]>0) //needs to go down
		setpoint.z_speed=-(MAX_SPEED-setpoint.x_speed-setpoint.y_speed);
		else
		setpoint.z_speed=0;		
	}
	setpoint.yaw=get_angle_with_XNED(board->estimation.state[0][1],board->estimation.state[1][1],board->estimation.state[2][1]); //Set yaw angle setpoint align with the speed
}