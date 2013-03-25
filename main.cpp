#include "lpc13.h"

int main(void)
{
	
	Lpc13 lpc;
	lpc.InitSystem();
	SystemTick sysTick= lpc.GetSystemTick();
	while(true)
	{
		sysTick.Delay(10);
	}
}
