/**************************************************************************/
/*! 
    @file     lpc13.cpp
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
#include "lpc13.h"
#include "Lpc13Pin.h"
#include "SSPSoftware.h"
#include "lpc13Timer.h"
#include "cmsis/LPC13Uxx.h"

Lpc13Timer Lpc13::GetTimer()
{
	//Init Timer
	Lpc13Timer t = Lpc13Timer();
	t.Init(10);
	return t;
}

Lpc13Pin Lpc13::GetPin(unsigned int port,unsigned int pin,PinType pinType)
{
	Lpc13Pin p = Lpc13Pin(port,pin,pinType);
	return p;
}

SSPSoftware Lpc13::GetSoftwareSSP(Pin* misoPin,Pin* mosiPin,Pin* sckPin,Timer* timer )
{
	
	SSPSoftware sspSoftware =   SSPSoftware(mosiPin,misoPin,sckPin,timer);
	return sspSoftware;
}

LPC13I2C Lpc13::GetI2C()
{
	LPC13I2C i2c = LPC13I2C();
	return i2c;
}

void Lpc13::Sleep()
{
	  LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;
 // LPC_SYSCON->PDSLEEPCFG = 0x0;
	SCB->SCR |= 0x04;
	LPC_PMU->PCON = 0x2;
	// Enter the sleep mode
  __WFI();
}

