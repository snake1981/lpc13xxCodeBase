#include "LPC13xx.h"


class SystemTick
{
	private:	
		static volatile uint32_t systickTicks; 
		static volatile uint32_t systickRollovers;
	public:
    static void Handler();
	  virtual void Init(uint32_t ticks);
	  virtual void Delay (uint32_t delayTicks);

};

