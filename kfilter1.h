#ifndef __KFILTER1_H__
#define __KFILTER1_H__

class KFilter1
{
public:
	KFilter1 (float var, float P, float varProcess);
	float filter(float reading);


private:
	float var;
	float P;
	float varProcess;
	float Xe;
};


#endif

