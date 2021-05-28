/* normal.h */

#ifndef NORMAL_H
#define NORMAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>
#include <math.h>

extern double normal (double m, double s)
{
	// Retorna uma variável pseudo-aleatória com distribuição normal
	static int iset = 0;
	static double gset;
	double fac,
	       r,
	       v1,
	       v2;

	if (!iset)
	{
		do
		{
			v1 = 2.0 * rand()/(RAND_MAX+1.0)-1.0;
			v2 = 2.0 * rand()/(RAND_MAX+1.0)-1.0;
			r = v1*v1 + v2*v2;
		} while (r>=1.0);

		fac = sqrt(-2.0*log(r)/r);
		gset = v1 * fac;
		iset = 1;
		return v2 * fac * s + m;
	}
	else
	{
		iset=0;
		return gset * s + m;
	}
}
#ifdef __cplusplus
};
#endif
#endif
