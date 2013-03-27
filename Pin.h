#ifndef _PIN_H_
#define _PIN_H_

class Pin
{
	public:
		virtual bool GetValue()=0;
		virtual void SetValue(bool value)=0;
};
#endif
