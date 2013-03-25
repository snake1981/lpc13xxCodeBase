#include "lpc13.h"

SystemTick Lpc13::GetSystemTick()
{
	return systemTick;
}

void Lpc13::InitSystem()
{
	//Init SysTimer
	systemTick= SystemTick();
	systemTick.Init(10);
}
