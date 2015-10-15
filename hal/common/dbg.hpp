/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file  	dbg.hpp
 * 
 * \author  MAV'RIC Team
 *   
 * \brief   Write debug messages
 *
 ******************************************************************************/

#ifndef DBG_H_
#define DBG_H_

#include "serial_dummy.hpp"
#include "console.hpp"
#include <stdint.h>

//namespace dbg{

template <typename Writeable>
class Dbg
{
public:
	static Dbg& getInstance()
	{
		static Dbg instance;
		return instance;
	}

	static bool init(Console<Writeable>* console)
	{
		getInstance().console_ = console;
		return true;
	}

	template<typename T>
	static bool write(T input)
	{
		Console<Writeable>* console = getInstance().console_;
		if(console){
			return console->write(input);
		}
		
		return false;
	}

	template<typename T>
	Dbg &operator<<(const T &a)
	{
		write(a);
		return *this;
	}

private:
	Console<Writeable>* console_;
	Dbg() : console_(0){}
};


	Dbg<Serial>& dbg = Dbg<Serial>::getInstance();
//}

typedef Dbg<Serial> Dbg_t;

#endif /* DBG_H_ */