/**************************************************************************/
/*! 
    @file     main.cpp
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
#include "SSPSoftware.h"
#include "SSP.h"
#include "I2C.h"

int main(void)
{
	Lpc13 lpc;
	Lpc13Timer timer= lpc.GetTimer();
	//Define Pins for spi
	Lpc13Pin mosiPin= lpc.GetPin(2,3,Input);
	Lpc13Pin misoPin= lpc.GetPin(2,2,Output);
	Lpc13Pin sckPin= lpc.GetPin(2,1,Input);
	//Chip select
  Lpc13Pin sSelPin= lpc.GetPin(2,0,Input);
	sSelPin.SetValue(false);
	
	SSPSoftware sspSoftware= lpc.GetSoftwareSSP(&misoPin,&mosiPin,&sckPin,&timer);
   
	SSP* ssp = &sspSoftware;
	
	ssp->SSPInit();
	unsigned char status1 = ssp->SSPSend(0x00);
	unsigned char status2 = ssp->SSPSend(0x00);
	

	LPC13I2C lpci2c = lpc.GetI2C();
  lpci2c.Init();
	
	uint8_t temp[2];

  bool result = lpci2c.Read(0x90,temp,2);
	while(true)
	{
		timer.DelayMS(10);
	}
}
