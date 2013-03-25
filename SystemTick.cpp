/**************************************************************************/
/*! 
    @file     SystemTick.cpp
    @author   F.Eisele
    @date     25.03.2013
    @version  1.0

    Software License Agreement (BSD License)

    Copyright (c) 2013, Felix Eisele
    All rights reserved.
		
		Code is based on K. Townsend (microBuilder.eu)

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
#include "SystemTick.h"

volatile uint32_t SystemTick::systickTicks = 0;
volatile uint32_t SystemTick::systickRollovers=0;

/**************************************************************************/
/*! 
    @brief Systick interrupt handler
*/
/**************************************************************************/
void SystemTick::Handler()
{
	
	SystemTick::systickTicks++;
	 // Increment rollover counter
  if (SystemTick::systickTicks == 0xFFFFFFFF) SystemTick::systickRollovers++;
	
}

/**************************************************************************/
/*! 
    @brief Initalize the SystemTickTimer
*/
/**************************************************************************/
void SystemTick::Init(uint32_t delayMs)
{ 
	SysTick_Config((SystemCoreClock / 1000)*delayMs);

}

/**************************************************************************/
/*! 
    @brief Delayfunction
*/
/**************************************************************************/
void SystemTick::Delay (uint32_t delayTicks) 
{
  uint32_t curTicks;
  curTicks = systickTicks;

  // Make sure delay is at least 1 tick in case of division, etc.
  if (delayTicks == 0) delayTicks = 1;

  if (curTicks > 0xFFFFFFFF - delayTicks)
  {
    // Rollover will occur during delay
    while (systickTicks >= curTicks)
    {
      while (systickTicks < (delayTicks - (0xFFFFFFFF - curTicks)));
    }      
  }
  else
  {
    while ((systickTicks - curTicks) < delayTicks);
  }
}


extern "C" {
	/**************************************************************************/
/*! 
    @brief Systick interrupt handler
*/
/**************************************************************************/
	void SysTick_Handler()
	{
		SystemTick::Handler();
		
	}
	
}
