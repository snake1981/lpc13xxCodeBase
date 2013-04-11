/**************************************************************************/
/*! 
    @file     lpc13I2C.cpp
    @author   F.Eisele
    @date     25.03.2013
    @version  1.0
    Copyright (c) 2013, Felix Eisele
    All rights reserved.

Partially based on NXP LPC13xx Code for 2i2.c File
*/
/**************************************************************************/
#include "lpc13I2C.h"

		//Define private vars
		volatile I2CState LPC13I2C::state;
		volatile uint8_t LPC13I2C::WriteBuffer[10];
		volatile uint8_t LPC13I2C::ReadBuffer[10];
		volatile uint32_t LPC13I2C::ReadIndex;
	  volatile uint32_t LPC13I2C::WriteIndex;
	  volatile uint32_t LPC13I2C::WriteLength;
	  volatile uint32_t LPC13I2C::ReadLength;

	/**************************************************************************/
		/*! 
				@brief init the i2c
		*/
		/**************************************************************************/


bool LPC13I2C::Init(void)
{
  /* It seems to be bit0 is for I2C, different from
  UM. To be retested along with SSP reset. SSP and I2C
  reset are overlapped, a known bug, for now, both SSP 
  and I2C use bit 0 for reset enable. Once the problem
  is fixed, change to "#if 1". */
#if 1
  LPC_SYSCON->PRESETCTRL |= (0x1<<1);
#else
  LPC_SYSCON->PRESETCTRL |= (0x1<<0);
#endif
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<5);
  LPC_IOCON->PIO0_4 &= ~0x3F;	/*  I2C I/O config */
  LPC_IOCON->PIO0_4 |= 0x01;		/* I2C SCL */
  LPC_IOCON->PIO0_5 &= ~0x3F;	
  LPC_IOCON->PIO0_5 |= 0x01;		/* I2C SDA */

  /*--- Clear flags ---*/
  LPC_I2C->CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;    


  LPC_I2C->SCLL   = I2SCLL_SCLL;
  LPC_I2C->SCLH   = I2SCLH_SCLH;



  /* Enable the I2C Interrupt */
  NVIC_EnableIRQ(I2C_IRQn);

  LPC_I2C->CONSET = I2CONSET_I2EN;
	return true;
};
  /**************************************************************************/
		/*! 
				@brief Send data over i2c
				@return true if success else false
		*/
		/**************************************************************************/

bool LPC13I2C::Write(uint8_t addr,const uint8_t* data, uint32_t len )
{
	WriteLength = len+2;
  ReadLength = 0;
	
	WriteBuffer[0] = addr;
  WriteBuffer[1] = 0x00;		// address 
	
	for(int x=0;x<len;x++)
	{
		WriteBuffer[x+2]=*data++;
	}

	I2CState st = Engine();
	if (st==I2CSTATE_ERROR)
		return false;
	else 
		return true;
};
	 /**************************************************************************/
		/*! 
				@brief Read data over i2c
	      @return true if success else false
		*/
bool LPC13I2C::Read(uint8_t addr, uint8_t* dataOut, uint32_t len )
{
	WriteLength = 1;
  ReadLength = len;
  WriteBuffer[0] = addr | 0x01; //read bit
	
  I2CState st = Engine();
	if (st==I2CSTATE_ERROR)
		return false;
	
	for(int x=0;x<len;x++)
	{
		*dataOut=ReadBuffer[x];
		dataOut++;
	}
	
	
		return true;
}

bool LPC13I2C::Start()
{
	 uint32_t timeout = 0;
  
  /*--- Issue a start condition ---*/
  LPC_I2C->CONSET = I2CONSET_STA;	/* Set Start flag */
  
  while((state != I2CSTATE_PENDING) && (timeout < MAXTIMEOUT))
  {
    timeout++;
  }
  
  return (timeout < MAXTIMEOUT);
}

bool LPC13I2C::Stop()
{
	uint32_t timeout = 0;
  
	 
	LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
  LPC_I2C->CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
  
  /*--- Wait for STOP detected ---*/
  while((LPC_I2C->CONSET & I2CONSET_STO) && (timeout < MAXTIMEOUT))
  {
    timeout++;
  }
  return (timeout >= MAXTIMEOUT);
}
I2CState LPC13I2C::Engine()
{
	 state = I2CSTATE_IDLE;
  ReadIndex = 0;
  WriteIndex = 0;
  if ( Start() != true )
  {
	Stop();
	return ( I2CSTATE_ERROR );
  }

  /* wait until the state is a terminal state */
  while (state < 0x100);

  return ( state );
}

void LPC13I2C::I2CIrqHandler()
{
		uint8_t StatValue;

	/* this handler deals with master read and master write only */
	StatValue = LPC_I2C->STAT;
	switch ( StatValue )
	{
	case 0x08:
		/*
		 * A START condition has been transmitted.
		 * We now send the slave address and initialize
		 * the write buffer
		 * (we always start with a write after START+SLA)
		 */
		WriteIndex = 0;
		LPC_I2C->DAT = WriteBuffer[WriteIndex++];
		LPC_I2C->CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
		state = I2CSTATE_PENDING;
		break;
	
	case 0x10:
		/*
		 * A repeated START condition has been transmitted.
		 * Now a second, read, transaction follows so we
		 * initialize the read buffer.
		 */
		ReadIndex = 0;
		/* Send SLA with R bit set, */
		LPC_I2C->DAT = WriteBuffer[WriteIndex++];
		LPC_I2C->CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
	break;
	
	case 0x18:
		/*
		 * SLA+W has been transmitted; ACK has been received.
		 * We now start writing bytes.
		 */
		   
		LPC_I2C->DAT = WriteBuffer[WriteIndex++];
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		break;

	case 0x20:
		/*
		 * SLA+W has been transmitted; NOT ACK has been received.
		 * Send a stop condition to terminate the transaction
		 * and signal I2CEngine the transaction is aborted.
		 */
	
		LPC_I2C->CONSET = I2CONSET_STO;
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		state = I2CSTATE_SLA_NACK;
		break;

	case 0x28:
		/*
		 * Data in I2DAT has been transmitted; ACK has been received.
		 * Continue sending more bytes as long as there are bytes to send
		 * and after this check if a read transaction should follow.
		 */
		if ( WriteIndex < WriteLength )
		{
				
			/* Keep writing as long as bytes avail */
		 LPC_I2C->DAT = WriteBuffer[WriteIndex++];
		}
		else
		{
			if ( ReadLength != 0 )
			{
				/* Send a Repeated START to initialize a read transaction */
				/* (handled in state 0x10)                                */
				LPC_I2C->CONSET = I2CONSET_STA;	/* Set Repeated-start flag */
			}
			else
			{
				state = I2CSTATE_ACK;
				LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
			}
		}
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		break;

	case 0x30:
		/*
		 * Data byte in I2DAT has been transmitted; NOT ACK has been received
		 * Send a STOP condition to terminate the transaction and inform the
		 * I2CEngine that the transaction failed.
		 */
		LPC_I2C->CONSET = I2CONSET_STO;
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		state = I2CSTATE_NACK;
		break;

	case 0x38:
		/*
		 * Arbitration loss in SLA+R/W or Data bytes.
		 * This is a fatal condition, the transaction did not complete due
		 * to external reasons (e.g. hardware system failure).
		 * Inform the I2CEngine of this and cancel the transaction
		 * (this is automatically done by the I2C hardware)
		 */
		state = I2CSTATE_ARB_LOSS;
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		break;

	case 0x40:
		/*
		 * SLA+R has been transmitted; ACK has been received.
		 * Initialize a read.
		 * Since a NOT ACK is sent after reading the last byte,
		 * we need to prepare a NOT ACK in case we only read 1 byte.
		 */
		if ( ReadLength == 1 )
		{
			/* last (and only) byte: send a NACK after data is received */
			LPC_I2C->CONCLR = I2CONCLR_AAC;
		}
		else
		{
			/* more bytes to follow: send an ACK after data is received */
			LPC_I2C->CONSET = I2CONSET_AA;
		}
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		break;

	case 0x48:
		/*
		 * SLA+R has been transmitted; NOT ACK has been received.
		 * Send a stop condition to terminate the transaction
		 * and signal I2CEngine the transaction is aborted.
		 */
		LPC_I2C->CONSET = I2CONSET_STO;
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		state = I2CSTATE_SLA_NACK;
		break;

	case 0x50:
		/*
		 * Data byte has been received; ACK has been returned.
		 * Read the byte and check for more bytes to read.
		 * Send a NOT ACK after the last byte is received
		 */
		ReadBuffer[ReadIndex++] = LPC_I2C->DAT;
		if ( ReadIndex < (ReadLength-1) )
		{
			/* lmore bytes to follow: send an ACK after data is received */
			LPC_I2C->CONSET = I2CONSET_AA;
		}
		else
		{
			/* last byte: send a NACK after data is received */
			LPC_I2C->CONCLR = I2CONCLR_AAC;
		}
		LPC_I2C->CONCLR = I2CONCLR_SIC;
		break;
	
	case 0x58:
		/*
		 * Data byte has been received; NOT ACK has been returned.
		 * This is the last byte to read.
		 * Generate a STOP condition and flag the I2CEngine that the
		 * transaction is finished.
		 */
		ReadBuffer[ReadIndex++] = LPC_I2C->DAT;
		state = I2CSTATE_ACK;
		LPC_I2C->CONSET = I2CONSET_STO;	/* Set Stop flag */
		LPC_I2C->CONCLR = I2CONCLR_SIC;	/* Clear SI flag */
		break;

        
        /* Slave Mode */

//      case 0x60:					/* An own SLA_W has been received. */
//	case 0x70:
//                RdIndex = 0;
//                I2C_I2CCONSET = I2CONSET_AA;            /* assert ACK after SLV_W is received */
//                I2C_I2CCONCLR = I2CONCLR_SIC;
//                I2CSlaveState = I2C_WR_STARTED;
//                break;
//	
//	case 0x80:					/*  data receive */
//	case 0x90:
//                if ( I2CSlaveState == I2C_WR_STARTED )
//                {
//                  I2CRdBuffer[RdIndex++] = I2C_I2CDAT;
//                  I2C_I2CCONSET = I2CONSET_AA;          /* assert ACK after data is received */
//                }
//                else
//                {
//                  I2C_I2CCONCLR = I2CONCLR_AAC;         /* assert NACK */
//                }
//                // ToDo: Set a flag to indicate data is ready and process it somewhere
//                I2C_I2CCONCLR = I2CONCLR_SIC;
//                break;
//		
//	case 0xA8:					/* An own SLA_R has been received. */
//	case 0xB0:
//                // RdIndex = 0;
//                WrIndex = I2CRdBuffer[0];               /* The 1st byte is the index. */
//                I2C_I2CDAT = I2CRdBuffer[WrIndex+1];    /* write the same data back to master */
//                WrIndex++;				/* Need to skip the index byte in RdBuffer */
//                I2C_I2CCONSET = I2CONSET_AA;            /* assert ACK after SLV_R is received */
//                I2C_I2CCONCLR = I2CONCLR_SIC;
//                I2CSlaveState = I2C_RD_STARTED;
//                break;
//	
//	case 0xB8:					/* Data byte has been transmitted */
//	case 0xC8:
//                if ( I2CSlaveState == I2C_RD_STARTED )
//                {
//                  I2C_I2CDAT = I2CRdBuffer[WrIndex+1];  /* write the same data back to master */
//                  WrIndex++;                            /* Need to skip the index byte in RdBuffer */
//                  I2C_I2CCONSET = I2CONSET_AA;          /* assert ACK  */
//                }
//                else
//                {
//                  I2C_I2CCONCLR = I2CONCLR_AAC;         /* assert NACK  */
//                }	
//                I2C_I2CCONCLR = I2CONCLR_SIC;
//                break;
//
//	case 0xC0:					/* Data byte has been transmitted, NACK */
//                I2C_I2CCONCLR = I2CONCLR_AAC;		/* assert NACK  */
//                I2C_I2CCONCLR = I2CONCLR_SIC;
//                I2CSlaveState = DATA_NACK;
//                break;
//
//	case 0xA0:					/* Stop condition or repeated start has */
//                I2C_I2CCONSET = I2CONSET_AA;            /* been received, assert ACK.  */
//                I2C_I2CCONCLR = I2CONCLR_SIC;
//                I2CSlaveState = I2C_IDLE;
//                break;

	default:
                LPC_I2C->CONCLR = I2CONCLR_SIC;
//                if (_I2cMode = I2CSLAVE)
//                {
//                  I2C_I2CCONSET = I2CONSET_I2EN | I2CONSET_SI;
//                }
                break;
  }
  return;
}

extern "C" {
	/**************************************************************************/
/*! 
    @brief I2C interrupt handler
*/
/**************************************************************************/
	void I2C_IRQHandler()
	{
		LPC13I2C::I2CIrqHandler();
		
	}
	
}
