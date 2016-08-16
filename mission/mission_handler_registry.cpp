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
 * \file mission_handler_registry.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The mission handler registry is responsible for registering and
 *        obtaining mission handlers based on an inputted waypoint
 *
 ******************************************************************************/


#include "mission/mission_handler_registry.hpp"

#include "mission/mission_handler.hpp"
 
extern "C"
{
}

//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_handler_registry::Mission_handler_registry() :
		registered_mission_handler_count_(0)
{

}

bool Mission_handler_registry::register_mission_handler(Mission_handler& handler)
{
    // Check for maximum count
    if (MAX_REGISTERED_MISSION_HANDLERS == registered_mission_handler_count_)
    {
        print_util_dbg_print("[MISSION_HANDLER_REGISTRY]: Too many registed mission handlers\r\n");
        return false;
    }

    // Check for duplicates
    for (uint8_t i = 0; i < registered_mission_handler_count_; i++)
    {
        if (registered_mission_handlers_[i] == &handler)
        {
            print_util_dbg_print("[MISSION_HANDLER_REGISTRY]: Mission handler already registed\r\n");
            return false;
        }
    }

    // Register mission
    registered_mission_handlers_[registered_mission_handler_count_] = &handler;
    registered_mission_handler_count_++;
    print_util_dbg_print("[MISSION_HANDLER_REGISTRY]: Mission handler registered\r\n");
    return true;
}

Mission_handler* Mission_handler_registry::get_mission_handler(const Waypoint& waypoint)
{
	// Search for the first handler than can accept this waypoint
    for (uint8_t i = 0; i < registered_mission_handler_count_; i++)
    {
        // Check if proper handler
        if (registered_mission_handlers_[i]->can_handle(waypoint))
        {
        	return registered_mission_handlers_[i];
        }
    }

    print_util_dbg_print("[MISSION_HANDLER_REGISTRY]: Handler could not be found\r\n");
    return NULL;
}