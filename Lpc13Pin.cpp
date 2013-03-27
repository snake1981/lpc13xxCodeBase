#include "Lpc13Pin.h"

bool Lpc13Pin::GetValue()
{
	bool value = (gpio->DATA >> this->pin) & 1;
	return value;
}

void Lpc13Pin::SetValue(bool value)
{
	if(value)
		gpio->DATA |= 1<<this->pin;
	else
			gpio->DATA &= ~(1<<pin);
	//gpio->MASKED_ACCESS[(1<<this->pin)] = (value<<this->pin);
}

Lpc13Pin::Lpc13Pin(unsigned int port,unsigned int pin,PinType pinType)
{
	this->port=port;
	this->pin=pin;
	
	if (this->port==0)
		this->gpio = LPC_GPIO0;
	else if (this->port==1)
		this->gpio = LPC_GPIO1;
	else if (this->port==2)
		this->gpio = LPC_GPIO2;
	else if (this->port==3)
		this->gpio = LPC_GPIO3;
	
	if (pinType==Input)
	{
		gpio->DIR |= 1<<this->pin;
	}
	else if (pinType==Input)
	{
		gpio->DIR &= ~(1<<pin);
	}
	else if (pinType==InOutput)
	{
		gpio->DIR |= 1<<this->pin;
	}
	

}
