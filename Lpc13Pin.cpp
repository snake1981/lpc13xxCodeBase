#include "Lpc13Pin.h"

bool Lpc13Pin::GetValue()
{
	
	
	bool value  = false;
	if (this->port==0)
		value=	(gpio->PIN[0] >> this->pin) & 1;
	else if (this->port==1)
		value=	(gpio->PIN[1] >> this->pin) & 1;
	

	return value;
}

void Lpc13Pin::SetValue(bool value)
{

		if (this->port==0)
		{
			
			if (value)
				gpio->PIN[0] |= 1<<this->pin;
			else
				gpio->PIN[0] &= ~ 1<<this->pin;
		}
		else if (this->port==1)
		{
			if (value)
				gpio->PIN[1] |= 1<<this->pin;
			else
				gpio->PIN[1] &= ~ 1<<this->pin;
		}
	
	
}

Lpc13Pin::Lpc13Pin(unsigned int port,unsigned int pin,PinType pinType)
{
	this->port=port;
	this->pin=pin;
	
//	if (this->port==0)
		this->gpio = LPC_GPIO;

	
	if (port==0)
	{
		if (pinType==Input)
		{
			gpio->DIR[0] |= 1<<this->pin;
		}
	else if (pinType==Output)
		{
			gpio->DIR[0]  &= ~(1<<pin);
		}
	else if (pinType==InOutput)
		{
			gpio->DIR[0]  |= 1<<this->pin;
		}
	}
	else if(port==1)
	{
			if (pinType==Input)
		{
			gpio->DIR[1] |= 1<<this->pin;
		}
	else if (pinType==Input)
		{
			gpio->DIR[1]  &= ~(1<<pin);
		}
	else if (pinType==InOutput)
		{
			gpio->DIR[1]  |= 1<<this->pin;
		}
	}
	
	
	

}
