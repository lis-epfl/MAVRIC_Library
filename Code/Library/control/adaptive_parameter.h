/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file adaptive_parameter.h
 * 
 * This file automatically adapts the existing parameters based on given control variable
 *
 * Usage example:
 * // control variable
 * float control = 0.0;
 * // param1
 * float param1 = 0.0;
 * int number_set_points1 = 5;
 * float new_setpoints1[5] = {0.1, 0.2, 0.4, 0.7, 0.9};
 * float new_setvalues1[5] = {0, 1, 0, 100, 0};
 * add_adaptive_parameter(&control, &param1, number_set_points1, new_setpoints1, new_setvalues1);
 * // param2
 * float param2 = 0.0;
 * int number_set_points2 = 2;
 * float new_setpoints2[2] = {0.1, 1.5};
 * float new_setvalues2[2] = {10, -10};
 * add_adaptive_parameter(&control, &param2, number_set_points2,
 * 						new_setpoints2, new_setvalues2);
 * printf("Control, Param1, Param2\n");
 * int i;
 * for (i = 0; i < 20; ++i)
 * {
 *		control = 1 / 20.0 * i;
 *		update_all_adaptive_parameters();
 *		printf("%f, %f, %f\n", control, param1, param2);
 * }
 */


#ifndef ADAPTIVE_PARAMETER_H_
#define ADAPTIVE_PARAMETER_H_

#define MAX_ADAPT_PARAM_SETPOINTS 5
#define MAX_ADAPT_PARAM_COUNT 10

typedef struct 
{
	float* control_variable;					///< Control Variable
	float* parameter;							///< Parameter
	int nb_setpoints;							///< Number of setpoints
	float setpoints[MAX_ADAPT_PARAM_SETPOINTS];	///< set points
	float setvalues[MAX_ADAPT_PARAM_SETPOINTS];	///< set values
} Adaptive_Parameter_t;

typedef struct
{
	int param_count;										///< Number of Parameters
	Adaptive_Parameter_t parameters[MAX_ADAPT_PARAM_COUNT];	///< Parameter set
} Adaptive_Parameter_Set_t;

/**
 * \brief					Returns a pointer to the adaptive parameter set
 */
Adaptive_Parameter_Set_t* get_param_set(void);

/**
 * \brief					Initializes the adaptive parameter set
 */
void init_adaptive_parameters(void);

/**
 * \brief					Add an adaptive parameter to the parameter set
 *
 * \param control_variable	 
 * \param parameter
 * \param nb_setpoints
 * \param setpoints
 * \param setvalues
 *
 * \return					Returns 1 if successfully added and 0 otherwise
 */
int add_adaptive_parameter(float* control_variable, float* parameter, 
							int nb_setpoints, float* setpoints, float* setvalues);
/**
 * \brief					Update adaptive parameter
 *
 * \param	param			Adaptive parameter to be updated  
 */							
void update_adaptive_parameter(Adaptive_Parameter_t param);

/**
 * \brief					Update all adaptive parameters
 */	
void update_all_adaptive_parameters(void);

#endif /* ADAPTIVE_PARAMETER_H_ */
