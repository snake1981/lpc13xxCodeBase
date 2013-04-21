#ifndef LM75_h
#define LM75_h

#include "../../type.h"
#include "../../i2c.h"

class LM75 {
	private:
		I2C* i2c;
	  uint8_t address;
  public:
    LM75 (I2C*,uint8_t address);
    uint8_t ReadTemp (void);
    void Shutdown (bool);
};


#endif
