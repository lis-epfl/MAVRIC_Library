 /* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file boardsupport.h
 *
 *  Initialization of all hardware related elements (communication lines, sensors devices, etc)
 */


#ifndef BOARDSUPPORT_H_
#define BOARDSUPPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "central_data.h"

#define BOARD USER_BOARD

/**
 * \brief	Initialize the hardware related elements (communication lines, sensors devices, etc)
 *
 * \param	centralData		The pointer to the structure where all central data is stored
 */
void initialise_board(central_data_t* centralData);

#ifdef __cplusplus
}
#endif

#endif /* BOARDSUPPORT_H_ */