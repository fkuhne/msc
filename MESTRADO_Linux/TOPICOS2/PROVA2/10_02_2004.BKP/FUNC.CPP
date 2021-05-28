/* func.cpp */

#include "normal.h"
#include "mmath.h"
#include <math.h>

#define n 3

CVECTOR<double> f(const CVECTOR<double> q, const CVECTOR<double> u, const double T)
{
	// Calcula q[k+1] = f(q[k],u[k]).
	CVECTOR<double> B(n);
	B[0] = q[0] + u[0] * cos(q[2]) * T;
	B[1] = q[1] + u[0] * sin(q[2]) * T;
	B[2] = q[2] + u[1] * T;
	return B;
}


MATRIX<double> jacob_F(const CVECTOR<double> q, const CVECTOR<double> u, const double T)
{
	// Calcula o jacobiano de f(^q[k|k]):
	MATRIX<double> F(n,n);
	F.eye();
	F[0][2] = -sin(q[2]) * u[0] * T;
	F[1][2] = cos(q[2]) * u[0] * T;
	return F;
}

CVECTOR<double> noise(const double sigma_1, const double sigma_2, const double sigma_3)
{
	CVECTOR<double> noise_vec(n);
	noise_vec[0] = normal(0,sigma_1);
	noise_vec[1] = normal(0,sigma_2);
	noise_vec[2] = normal(0,sigma_3);
	return noise_vec;
}
