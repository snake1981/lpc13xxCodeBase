/**************************************************************************/
/*! 
    @file     lpc13I2C.h
    @author   F.Eisele
    @date     25.03.2013
    @version  1.0

    Copyright (c) 2013, Felix Eisele
    All rights reserved.
		
		Partially based on NXP LPC13xx Code for 2i2.h

 
*/
/**************************************************************************/
#ifndef _LPC13I2C_H
#define _LPC13I2C_H

#include "LPC13Uxx.h"
#include "i2c.h"


enum I2CState
{
	I2CSTATE_IDLE=       0x000,
 I2CSTATE_PENDING=    0x001,
 I2CSTATE_ACK=        0x101,
 I2CSTATE_NACK=       0x102,
 I2CSTATE_SLA_NACK=   0x103,
 I2CSTATE_ARB_LOSS=   0x104,
 I2CSTATE_ERROR= 0x105
};

class LPC13I2C: public I2C
{
	private:
		/* I2C Control Set Register */
		static const uint32_t I2CONSET_I2EN=0x00000040;
		static const uint32_t I2CONSET_SI=0x00000008;
		static const uint32_t I2CONSET_AA=0x00000004;
		static const uint32_t I2CONSET_STO=0x00000010;
		static const uint32_t I2CONSET_STA=0x00000020;
	
		/* I2C Control clear Register */
		static const uint32_t I2CONCLR_AAC=0x00000004; /* Assert acklnowedge clear bit*/
		static const uint32_t I2CONCLR_SIC=0x00000008; /* I2C interrupt clear bit */
		static const uint32_t I2CONCLR_STAC=0x00000020; /* START flag clear bit */
		static const uint32_t I2CONCLR_I2ENC=0x00000040; /* I2C interface disable bit */
	
	
		static const uint32_t I2SCLH_SCLH=0x00000180;  /* I2C SCL Duty Cycle High Reg */
		static const uint32_t I2SCLL_SCLL=0x00000180;/* I2C SCL Duty Cycle Low Reg */
	
	
		//Max Timeout
		static const uint32_t MAXTIMEOUT= 0x00FFFFFF;
	
		//actual state of i2c
		volatile static I2CState state;
		volatile static uint8_t WriteBuffer[10];
		volatile static uint8_t ReadBuffer[10];
	  volatile static uint32_t ReadIndex;
	  volatile static uint32_t WriteIndex;
	  volatile static uint32_t WriteLength;
	  volatile static uint32_t ReadLength;
		
		bool Start();
		bool Stop();
		I2CState Engine();
	
	public:
			static void I2CIrqHandler(void);
		/**************************************************************************/
		/*! 
				@brief init the i2c
		*/
		/**************************************************************************/
		virtual bool Init( void );
	  /**************************************************************************/
		/*! 
				@brief Send data over i2c
				@return true if success else false
		*/
		/**************************************************************************/
		virtual bool Write(uint8_t addr, const uint8_t* data, uint32_t len );
	 /**************************************************************************/
		/*! 
				@brief Read data over i2c
	      @return true if success else false
		*/
		/**************************************************************************/
	  virtual bool Read(uint8_t addr, uint8_t* dataOut, uint32_t len );
		
		 
};

#endif
