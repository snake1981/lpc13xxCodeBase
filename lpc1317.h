/**************************************************************************/
/*! 
    @file     PWM.h
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
#ifndef _LPC1317_H
#define _LPC1317_H

#define SYSAHBCLKCTRL_CT16B0                  		((unsigned int) 0x00000080)
#define SYSAHBCLKCTRL_CT16B1                  		((unsigned int) 0x00000100)

#define TMR16B0EMR_EM0 												((unsigned int) 0x00000001)
#define TMR16B0EMR_EMC0_TOGGLE            		((unsigned int) 0x00000030)
#define TMR16B0EMR_EM1 												((unsigned int) 0x00000002)
#define TMR16B0EMR_EMC1_TOGGLE            		((unsigned int) 0x000000C0)

#define TMR16B0EMR_EM2 												((unsigned int) 0x00000004)
#define TMR16B0EMR_EMC2_TOGGLE            		((unsigned int) 0x00000600)

#define TMR16B0PWMC_PWM0_ENABLED              ((unsigned int) 0x00000001)
#define TMR16B0PWMC_PWM1_ENABLED              ((unsigned int) 0x00000002)
#define TMR16B0PWMC_PWM2_ENABLED              ((unsigned int) 0x00000004)
#define TMR16B0PWMC_PWM3_ENABLED              ((unsigned int) 0x00000008)
#define TMR16B0MCR_MR1_RESET_ENABLED          ((unsigned int) 0x00000400)


#endif
