/**************************************************************************/
/*! 
    @file     SSpSoftware.h
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

#ifndef _SSPSOFTWARE_H
#define _SSPSOFTWARE_H

#include "base/SSP.h"
#include "Lpc13Pin.h"
#include "Lpc13Timer.h"

class SSPSoftware : public SSP
{
	private:
		//Write one byte to ssp
		unsigned char WriteSSP(unsigned char value);
	  // Pin for MOSI
		Pin* MOSIPin;
		//Pin for SCK
		Pin* SCKPin;
		//Pin for MISO
		Pin* MISOPin;
		Timer* timer;
	public:
		/**************************************************************************/
		/*! 
				@brief ctor
		*/
		/**************************************************************************/
		SSPSoftware(Pin* mosiPin,Pin* misoPin,Pin* sckPin,Timer* timer);
	  	/**************************************************************************/
			/*! 
					@brief init the ssp
			*/
			/**************************************************************************/
		virtual void SSPInit( void );
	  	  /**************************************************************************/
		/*! 
				@brief Send one byte 
		*/
		/**************************************************************************/
		virtual unsigned char SSPSend(unsigned char value );
};

#endif
