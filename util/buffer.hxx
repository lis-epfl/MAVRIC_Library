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
 * \file buffer.cpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief Buffer
 *
 ******************************************************************************/

#ifndef BUFFER_HXX_
#define BUFFER_HXX_


template<uint32_t S, typename T>
Buffer_tpl<S, T>::Buffer_tpl(void):
	head_(0),
	tail_(0)
{}


template<uint32_t S, typename T>
bool Buffer_tpl<S, T>::put_lossy(const T& data) 
{
	uint32_t tmp;
	
	tmp = (head_ + 1)%(S+1);

	if( tmp == tail_ ) 
	{
		// error: receive buffer overflow!!
		// lose old incoming data at the end of the buffer
		tail_ = (tail_ + 1)%(S+1);
	}

	// store incoming data in buffer
	buffer_[head_] = data;
	head_ = tmp;
	
	return true;
}


template<uint32_t S, typename T>
bool Buffer_tpl<S, T>::put(const T& data) 
{
	uint32_t tmp;
	tmp = (head_ + 1)%(S+1);

	if( tmp == tail_ ) 
	{
		// error: buffer full!
		return false;
	}
	else
	{
		// store incoming data in buffer
		buffer_[head_] = data;
		head_ = tmp;
		
		return true;
	}
}


template<uint32_t S, typename T>
bool Buffer_tpl<S, T>::get(T& data) 
{
	bool ret = false;
	
	if (head_ != tail_)
	{
		data  = buffer_[tail_];
		tail_ = (tail_ + 1)%(S+1);
		ret = true;
	}

	return ret;
}


template<uint32_t S, typename T>
void Buffer_tpl<S, T>::clear(void) 
{
	head_ = 0;
	tail_ = 0;
}


template<uint32_t S, typename T>
uint32_t Buffer_tpl<S, T>::readable(void) const
{
	return (S + 1 + head_ - tail_)%(S+1);
}


template<uint32_t S, typename T>
uint32_t Buffer_tpl<S, T>::writeable(void) const
{
	return S - readable();
}


template<uint32_t S, typename T>
bool Buffer_tpl<S, T>::full(void) const
{
	return (((head_ + 1)%(S+1)) == tail_);
}


template<uint32_t S, typename T>
bool Buffer_tpl<S, T>::empty(void) const
{
	return (head_ == tail_);
}

#endif /* BUFFER_HXX_ */