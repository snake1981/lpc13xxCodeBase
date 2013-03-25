#include "SystemTick.h"

volatile uint32_t SystemTick::systickTicks = 0;
volatile uint32_t SystemTick::systickRollovers=0;

void SystemTick::Handler()
{
	
	SystemTick::systickTicks++;
	 // Increment rollover counter
  if (SystemTick::systickTicks == 0xFFFFFFFF) SystemTick::systickRollovers++;
	
}

void SystemTick::Init(uint32_t delayMs)
{ 
	SysTick_Config((SystemCoreClock / 1000)*delayMs);

}

void SystemTick::Delay (uint32_t delayTicks) 
{
  uint32_t curTicks;
  curTicks = systickTicks;

  // Make sure delay is at least 1 tick in case of division, etc.
  if (delayTicks == 0) delayTicks = 1;

  if (curTicks > 0xFFFFFFFF - delayTicks)
  {
    // Rollover will occur during delay
    while (systickTicks >= curTicks)
    {
      while (systickTicks < (delayTicks - (0xFFFFFFFF - curTicks)));
    }      
  }
  else
  {
    while ((systickTicks - curTicks) < delayTicks);
  }
}


extern "C" {
	void SysTick_Handler()
	{
		SystemTick::Handler();
		
	}
	
}
