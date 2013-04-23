/**************************************************************************/
/*! 
    @file     Timer.h
    @author   F.Eisele
    @date     25.03.2013
    @version  1.0

    Software License Agreement (BSD License)

    Copyright (c) 2013, Felix Eisele
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef _TIMER_H
#define _TIMER_H
#include "type.h"

/**************************************************************************/
/*! 
    @brief Baseclass for timer implementation
*/
/**************************************************************************/
class Timer
{
	public:
		/**************************************************************************/
		/*! 
				@brief Wait delayMs in milliseconds
		*/
		/**************************************************************************/
		virtual void DelayMS( uint32_t delayMs )=0;
			/**************************************************************************/
		/*! 
				@brief Wait delayMs in microseconds
		*/
		/**************************************************************************/
		virtual void DelayUS( uint32_t delayUs )=0;
	 
};

#endif
