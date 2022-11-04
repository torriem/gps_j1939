#include "kfilter1.h"

KFilter1::KFilter1 (float var, float P, float varProcess) : var(var), P(P), varProcess(varProcess)
{
	Xe = 0;
}

float KFilter1::filter(float reading)
{
	float Pc;
	float G;
	float Xp;
	float Zp;

	Pc = P + varProcess;
	G = Pc / (Pc + var);
	P = (1 - G) * Pc;
	Xp = Xe;
	Zp = Xp;
	Xe = (G * (reading - Zp)) + Xp;

	return Xe;
}
