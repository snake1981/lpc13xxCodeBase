/**************************************************************************/
/*! 
    @file     Lpc13PWM.h
    @author   F.Eisele
    @date     28.04.2013
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
#ifndef _LPC13PWM_H
#define _LPC13PWM_H

#include "cmsis/LPC13Uxx.h"
#include "base/PWM.h"
#include "lpc1317.h"

/**************************************************************************/
/*! 
    @brief PWM implementation on lpc1317 
					 at the moment only pwm for CT16B0
*/
/**************************************************************************/
class Lpc13PWM: public PWM
{
	private:
		  //channel number of the pwm
			uint8_t channel;
			//pulseWidth
			uint32_t pulseWidth;
			//dutycycle
			uint8_t dutyCycle;
			
	
	private:
		/**************************************************************************/
		/*! 
			@brief calculate the dutycycle value
		*/
		/**************************************************************************/
		static uint32_t CalculateDutyCycleValue(uint8_t dutyCycle,uint32_t pulseWidth   );
	public:
			/**************************************************************************/
			/*! 
				@brief CTOR
			*/
			/**************************************************************************/
		  Lpc13PWM(uint8_t channel);
			/**************************************************************************/
			/*! 
					@brief Initilation the PWM logic
			*/
			/**************************************************************************/
		  virtual void Init();
			/**************************************************************************/
			/*! 
					@brief start the pwm
			*/
			/**************************************************************************/
			virtual void Start();
			/**************************************************************************/
			/*! 
					@brief stop the pwm
			*/
			/**************************************************************************/
			virtual void Stop();
			/**************************************************************************/
			/*! 
					@brief set frequency (in us)
			*/
			/**************************************************************************/
			virtual void SetFrequency(uint16_t us );
			/**************************************************************************/
			/*! 
				@brief set dutycyle in percent
			*/
			/**************************************************************************/
			virtual void SetDutyCycle( uint8_t percentage );
};

#endif
