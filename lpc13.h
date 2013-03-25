#include "SystemTick.h"

class Lpc13
{
	private:
		SystemTick systemTick;
	public:
		void InitSystem();
		SystemTick GetSystemTick();
};
