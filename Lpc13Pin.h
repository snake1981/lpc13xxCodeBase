#ifndef _LPC13PIN_H
#define _LPC13PIN_H


#include "Pin.h"
#include "lpc13xx.h"

enum PinType {Input, Output,InOutput};

class Lpc13Pin: public Pin
{
	private:
		unsigned int port;
		unsigned int pin;
	  LPC_GPIO_TypeDef* gpio;
	public:
		Lpc13Pin(unsigned int port,unsigned int pin,PinType pinType);
		virtual bool GetValue();
		virtual void SetValue(bool value);
	
};

#endif
