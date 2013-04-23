/**************************************************************************/
/*! 
    @file     LM75.h
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

#ifndef LM75_h
#define LM75_h

#include "../../type.h"
#include "base/i2c.h"

//Registers
enum Registers
{
	//Temperature register
	Temp=0x00,
	//Configuration register
	Conf=0x01,
	//Thyst register
	Thyst=0x02,
	//Tos register
	Tos=0x03
};

class LM75 {
	private:
		//instance of the i2c interface
		I2C* i2c;
	  //Address of the sensor
	  uint8_t address; 
		// Write a 8bit value to the register of the sensor
	  void Write8BitRegister(uint8_t reg,uint8_t value);
	  // Read a 8bit value from register
		uint8_t Read8BitRegister(uint8_t reg);
	  // Read a 16bit value from register
	  uint16_t Read16BitRegister(uint8_t reg);
	
  public:
	  //ctor
    LM75 (I2C*,uint8_t address);
    //Read Temperature from the sensor
		float ReadTemp (void);
	  //Set the shutdownmode 
    void Shutdown (bool mode);
};


#endif
