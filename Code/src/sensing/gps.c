/*
  Copyright (C) 2008. LIS Laboratory, EPFL, Lausanne

  This file is part of Aeropic.

  Aeropic is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 2.1 of the License, or
  (at your option) any later version.

  Aeropic is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with Aeropic.  If not, see <http://www.gnu.org/licenses/>.
*/ 
/*!
*	\file gps.c
*	\brief Source file to interface with library external GPS modules.
*
*/

//----------
// Includes
//----------
#include "gps.h"


//-------------------
// Private variables
//-------------------

#if GPS_TYPE == GPS_TYPE_EXTERNAL
	void (*gps_getGPSDataFct)(gps_Data*) = 0;
#endif


//---------------------------------
// Public functions implementation
//---------------------------------

#if GPS_TYPE == GPS_TYPE_EXTERNAL
	void gps_SetExternalGPSDataFct(void (*func)(gps_Data*))
	{
		gps_getGPSDataFct = func;
	}

	void gps_ExternalGetGPSData(gps_Data* data)
	{
		if (gps_getGPSDataFct)
			gps_getGPSDataFct(data);
	}
#endif
