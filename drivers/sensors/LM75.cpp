#include "LM75.h"

LM75::LM75(I2C* i2c,uint8_t address)
{
	this->i2c = i2c;
	this->address=address;
}

uint8_t LM75::ReadTemp (void)
{
	
  i2c->Init();
	
	uint8_t temp[2];

  bool result = i2c->Read(address,temp,2);
	return temp[0];
}
