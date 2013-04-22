/**************************************************************************/
/*! 
    @file     LM75.cpp
    @author   F.Eisele
    @date     22.04.2013
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

#include "LM75.h"
//ctor
LM75::LM75(I2C* i2c,uint8_t address)
{
	this->i2c = i2c;
	this->address=address;
}
//Read Temperature from the sensor
float LM75::ReadTemp (void)
{
	float temp;
	uint16_t result;
	result = Read16BitRegister(Temp);
	
	temp=  ((float)(int)result / 32) / 8;
	return temp;
}

//Set the shutdownmode 
void LM75::Shutdown(bool mode)
{	
	uint8_t config =Read8BitRegister(Conf);
	if (mode)
		config|=0x01;
	else
		config&=~0x01;
	
	this->Write8BitRegister(Conf,config);
}

// Read a 8bit value from register
uint8_t LM75::Read8BitRegister(uint8_t reg)
{
	uint8_t dataIn[]={this->address,reg,this->address|0x01};
  uint8_t dataOut[1];


	this->i2c->Read( this->address,dataIn,2,dataOut,1);
  return dataOut[0];
}

// Read a 16bit value from register
uint16_t LM75::Read16BitRegister(uint8_t reg)
{
	uint8_t dataIn[]={this->address,reg,this->address|0x01};
  uint8_t dataOut[2];


	this->i2c->Read( this->address,dataIn,2,dataOut,2);
  uint16_t result;
  result= ((uint16_t)( dataOut[0] << 8 ) | dataOut[1]);
  return result;
}

// Write a 8bit value to the register of the sensor
void LM75::Write8BitRegister(uint8_t reg,uint8_t value)
{
	uint8_t dataIn[]={reg,value};
	this->i2c->Write(this->address,dataIn,2);
}
