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
#include "drivers/sensors/LM75.h"
#include "drivers/rf/rfm70.h"

const unsigned char tx_buf[17]={
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x78};

void Receive_Packet( rfm70 &radio){
	unsigned char len,i,chksum;
	unsigned char rx_buf[ RFM70_MAX_PACKET_LEN ];

	if( ! radio.receive_fifo_empty() ){
		do {
			len=radio.register_read( RFM70_CMD_R_RX_PL_WID );

			if( len <= RFM70_MAX_PACKET_LEN ){
				radio.buffer_read( RFM70_CMD_R_RX_PAYLOAD,rx_buf,len );
			} else {
				radio.register_write( RFM70_CMD_FLUSH_RX,0 );
			}
							
		} while( ! radio.receive_fifo_empty());
		
		chksum = 0;
		for(i=0;i<16;i++){
			chksum +=rx_buf[i]; 
		}
	}
}

void Send_Packet( 
   rfm70 &radio, unsigned char type, 
   const unsigned char* pbuf, unsigned char len ,Timer* timer
){
	radio.mode_transmit();
	radio.buffer_write( type, pbuf, len); 
   timer->DelayMS(100);	 	
   // radio.mode_receive(); 
}    

int main(void)
{
	
  uint8_t buffer[16];
	Lpc13 lpc;
	Lpc13Timer timer= lpc.GetTimer();
	//Define Pins for spi
	Lpc13Pin mosiPin= lpc.GetPin(1,22,Input);
	Lpc13Pin misoPin= lpc.GetPin(1,21,Output);
	Lpc13Pin sckPin= lpc.GetPin(1,20,Input);
	//Chip select
  Lpc13Pin sSelPin= lpc.GetPin(1,19,Input);
	
	//Chip select
  Lpc13Pin cePin= lpc.GetPin(0,23,Input);
	cePin.SetValue(false);
	
	Lpc13Pin irq= lpc.GetPin(0,16,Output);
	
//	sSelPin.SetValue(true);
	//sSelPin.SetValue(false);
	
	SSPSoftware sspSoftware= lpc.GetSoftwareSSP(&misoPin,&mosiPin,&sckPin,&timer);
   
	SSP* ssp = &sspSoftware;
	
	ssp->SSPInit();
	
	rfm70 rfm(ssp,&sSelPin,&cePin,&timer);
 bool irqValue= irq.GetValue();
	bool isOnline =rfm.is_present();
	rfm.init();
	 rfm.channel( 9 );

   Receive_Packet( rfm);


LPC13I2C lpcI2c = lpc.GetI2C();

	
LM75 lm75(&lpcI2c,0x90);
 uint8_t result =  lm75.ReadTemp();
 int x=0;
 	while(true)
	{
		unsigned char retransmit = rfm.retransmit_count();
 if (result || isOnline||irqValue||retransmit)
	x=1;

//   Send_Packet( rfm, RFM70_CMD_W_TX_PAYLOAD_NOACK, tx_buf, 17,&timer);
		irqValue= irq.GetValue();
 Receive_Packet( rfm);
//		timer.DelayMS(1000);
	}
}
