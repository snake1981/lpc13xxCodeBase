/**************************************************************************/
/*! 
    @file     SSpSoftware.cpp
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

#include "SSpSoftware.h"

/**************************************************************************/
/*! 
		@brief init the ssp
*/
/**************************************************************************/
void SSPSoftware::SSPInit( void )
{
	// CPHA=1 und CPOL=1 
	
  // MOSI = CLK = HIGH 
	this->MOSIPin->SetValue(true);
	this->SCKPin->SetValue(true);
}
/**************************************************************************/
/*! 
		@brief Write one byte to ssp
*/
/**************************************************************************/
unsigned char SSPSoftware::SSPSend(unsigned char value )
{
	return this->WriteSSP(value);
}
/**************************************************************************/
/*! 
		@brief Write one byte to ssp
*/
/**************************************************************************/
unsigned char SSPSoftware::WriteSSP( unsigned char value )
{
	  unsigned char i;
   for( i =0 ; i < 8; i++ ){
      this->MOSIPin->SetValue( value & 0x80  );  
      // shift next bit		 
      value = (value << 1);    
      
      this->SCKPin->SetValue( 1 );     
			// fetch MISO     
      value |= this->MISOPin->GetValue();     
      
     this->SCKPin->SetValue( 0 );
		 //TODO inset wait_us(1)  
  }
  return value;
}

//CTOR
SSPSoftware::SSPSoftware(Pin* mosiPin,Pin* misoPin,Pin* sckPin)
{
	this->MISOPin=misoPin;
	this->MOSIPin=mosiPin;
	this->SCKPin=sckPin;
}
