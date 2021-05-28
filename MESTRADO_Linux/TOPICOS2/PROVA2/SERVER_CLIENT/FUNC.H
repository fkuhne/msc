/* func.h */

#ifndef FUNC_H
#define FUNC_H

#include "normal.h"
#include "mmath.h"
#include "ServerSocket.h"
#include "ClientSocket.h"
#include "SocketException.h"
#include <stdio.h>
#include <string.h>

#define n 3
#define p 2

#define CHAR_SIZE 2000


extern CVECTOR<double> f(const CVECTOR<double> A,const CVECTOR<double> U,double T)
{
	// Calcula x[k+1] = f(x[k],u[k]).
	CVECTOR<double> B(n);
	B[0] = A[0] + U[0] * cos(A[2]) * T;
	B[1] = A[1] + U[0] * sin(A[2]) * T;
	B[2] = A[2] + U[1] * T;
	return B;
}


extern MATRIX<double> jacob_F(const CVECTOR<double> A, const CVECTOR<double> U, double T)
{
	// Calcula o jacobiano de f(^x[k|k]):
	MATRIX<double> F(n,n);

	F.eye();
	F[0][2] = -sin(A[2]) * U[0] * T;
	F[1][2] = cos(A[2]) * U[0] * T;

	return F;
}


extern CVECTOR<double> noise(const CVECTOR<double> S)
{
	CVECTOR<double> noise_vec(S.lin());
      for (int i=0; i<S.lin(); i++) noise_vec[i] = normal(0,S[i]);

	return noise_vec;
}


extern CVECTOR<double> rcv_x (std::string msg)
{
	CVECTOR<double> x(n);
	char carac [100];
	char *next;
	
	memset(&carac[0], 0, sizeof(carac));
	for(int i=0; i<=msg.size(); i++) carac[i] = msg[i];

	x[0] = strtod(carac, &next);
	x[1] = strtod(next, &next);
	x[2] = strtod(next, &next);

	return x;
}


extern CVECTOR<double> rcv_u (std::string msg)
{
	char carac [100];
	CVECTOR<double> u(p);
	double dummy;
	char *next;
	memset(&carac[0], 0, sizeof(carac));
	for(int i=0; i<=msg.size(); i++) carac[i] = msg[i];

	dummy = strtod(carac, &next);
	for(int i=1; i<3; i++) dummy = strtod(next, &next);
	u[0] = strtod(next, &next);
	u[1] = strtod(next, &next);

	return u;
}

#endif
