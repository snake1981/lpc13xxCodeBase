/**************************************************************************/
/*! 
    @file     Lpc13PWM.cpp
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

#include "Lpc13PWM.h"



/**************************************************************************/
/*! 
	@brief CTOR
*/
/**************************************************************************/
Lpc13PWM::Lpc13PWM(uint8_t channel)
{
	this->channel=channel;
	
	//Set default values
	this->dutyCycle=50;
	
	//Set frequence to 5kHz at 72Mhz
	this->pulseWidth=SystemCoreClock/5000;
																	 
}
/**************************************************************************/
/*! 
		@brief Initilation the PWM logic
*/
/**************************************************************************/
void Lpc13PWM::Init()
{
	//enable ct16b0 clock
	LPC_SYSCON->SYSAHBCLKCTRL |=  SYSAHBCLKCTRL_CT16B0; 
  	
  // Set pulse width
  LPC_CT16B0->MR3= this->pulseWidth;
	//Bit 10 MR3R Reset on MR3
	LPC_CT16B0->MCR = 0x400;
	
	// Set Pin Configuration
	
		if (channel==0)
		{
			LPC_IOCON->PIO1_13 |= 0x02;// IOCON_PIO1_9_FUNC_CT16B1_MAT0; 
			// Set duty cycle  
			LPC_CT16B0->MR1=CalculateDutyCycleValue(this->dutyCycle,this->pulseWidth);
			//External Match Register Settings for PWM 
			LPC_CT16B0->EMR= TMR16B0EMR_EMC0_TOGGLE | TMR16B0EMR_EM0;  
			//Enable PWM
			LPC_CT16B0->PWMC = TMR16B0PWMC_PWM0_ENABLED |TMR16B0PWMC_PWM3_ENABLED;
		}
		else if  (channel==1)
		{
			LPC_IOCON->PIO1_14 |= 0x02;
			// Set duty cycle  
		
			LPC_CT16B0->MR1=	(this->pulseWidth * (100 - this->dutyCycle)) / 100;//CalculateDutyCycleValue(this->dutyCycle,this->pulseWidth);
			//External Match Register Settings for PWM 
			LPC_CT16B0->EMR= TMR16B0EMR_EMC1_TOGGLE | TMR16B0EMR_EM1; 
			//Enable PWM
			LPC_CT16B0->PWMC = TMR16B0PWMC_PWM1_ENABLED |TMR16B0PWMC_PWM3_ENABLED;
		}
		else if  (channel==2)
		{
			LPC_IOCON->PIO1_15 |= 0x02;
			// Set duty cycle  
			LPC_CT16B0->MR1=CalculateDutyCycleValue(this->dutyCycle,this->pulseWidth);
			//External Match Register Settings for PWM 
			LPC_CT16B0->EMR= TMR16B0EMR_EMC2_TOGGLE | TMR16B0EMR_EM2;
			//Enable PWM
			LPC_CT16B0->PWMC = TMR16B0PWMC_PWM2_ENABLED |TMR16B0PWMC_PWM3_ENABLED;
		}

	//enable timer interrupt 
  NVIC_EnableIRQ(CT16B0_IRQn);
}
/**************************************************************************/
/*! 
			@brief start the pwm
*/
/**************************************************************************/
void Lpc13PWM::Start()
{
	//enable timer
	 LPC_CT16B0->TCR|=  0x01;
}

/**************************************************************************/
/*! 
			@brief stop the pwm
*/
/**************************************************************************/
void Lpc13PWM::Stop()
{
	 //disable timer 
  LPC_CT16B0->TCR &=~  0x01;
}

/**************************************************************************/
/*! 
			@brief set frequency (in us)
*/
/**************************************************************************/
void Lpc13PWM::SetFrequency(uint16_t us )
{
	
	//Calculate ticks
	uint32_t ticks = (((SystemCoreClock/LPC_SYSCON->SYSAHBCLKDIV) / 1000000) * us);
 
	 if (ticks > 0xFFFF)
		return;
	
	//set pulseWidth
	this->pulseWidth =ticks-1;
	
		//calculate new dutyCycle value
	uint32_t newDutyCyclevalue =  CalculateDutyCycleValue(this->dutyCycle,this->pulseWidth);
	if (channel==0)
		LPC_CT16B0->MR0=newDutyCyclevalue;
	else if (channel==1)
		LPC_CT16B0->MR1=newDutyCyclevalue;
	else if (channel==2)
		LPC_CT16B0->MR2=newDutyCyclevalue;
	
	//Set pulseWidth to MR3
	LPC_CT16B0->MR3=pulseWidth; 
}

/**************************************************************************/
/*! 
	@brief set dutycyle in percent
*/
/**************************************************************************/
void Lpc13PWM::SetDutyCycle( uint8_t percentage )
{
	this->dutyCycle=percentage;
	uint32_t value = CalculateDutyCycleValue( this->pulseWidth,this->dutyCycle);
	
	if (channel==0)
		LPC_CT16B0->MR0=value;
	else if (channel==1)
		LPC_CT16B0->MR1=value;
	else if (channel==2)
		LPC_CT16B0->MR2=value;
		
}
//Calculate the duty cycle
uint32_t Lpc13PWM::CalculateDutyCycleValue(uint8_t dutyCycle,uint32_t pulseWidth   )
{
	uint32_t value = pulseWidth * (100-dutyCycle)/100;
	return value;
}
