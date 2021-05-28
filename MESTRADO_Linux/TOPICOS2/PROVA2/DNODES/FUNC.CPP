/* func.cpp */

#include "mmath.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define n 3

CVECTOR<double> f(const CVECTOR<double> A,const CVECTOR<double> U,double T)
{
	// Calcula x[k+1] = f(x[k],u[k]).
	CVECTOR<double> B(n);
	B[0] = A[0] + U[0] * cos(A[2]) * T;
	B[1] = A[1] + U[0] * sin(A[2]) * T;
	B[2] = A[2] + U[1] * T;
	return B;
}


MATRIX<double> jacob_F(const CVECTOR<double> A, const CVECTOR<double> U, double T)
{
	// Calcula o jacobiano de f(^x[k|k]):
	MATRIX<double> F(n,n);

	F.eye();
	F[0][2] = -sin(A[2]) * U[0] * T;
	F[1][2] = cos(A[2]) * U[0] * T;

	return F;
}

CVECTOR<double> noise(const double var_1, const double var_2, const double var_3, const CVECTOR<double> noise_vec)
{

	
}