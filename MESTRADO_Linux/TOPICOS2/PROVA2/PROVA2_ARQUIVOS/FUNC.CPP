/* func.cpp */

#include "normal.h"
#include "mmath.h"
#include <math.h>

#define n 3
#define p 2
#define CHAR_SIZE 1000

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

CVECTOR<double> rcv_q (std::string msg)
{
	CVECTOR<double> q(n);
	char carac [100];
	char *next;

	memset(&carac[0], 0, sizeof(carac));
	for(int i=0; i<=msg.size(); i++) carac[i] = msg[i];

	q[0] = strtod(carac, &next);
	q[1] = strtod(next, &next);
	q[2] = strtod(next, &next);

	return q;
}


CVECTOR<double> rcv_u (std::string msg)
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

MATRIX<double> rcv_E (std::string msg)
{
      MATRIX<double> E(n,n);
      char rcv_err[CHAR_SIZE];
	char *next;

	memset(&rcv_err[0], 0, CHAR_SIZE);
	for(int i=0; i<=msg.size(); i++) rcv_err[i] = msg[i];

	E[0][0] = strtod(rcv_err, &next);
	E[0][1] = strtod(next, &next);
	E[0][2] = strtod(next, &next);
	E[1][0] = strtod(next, &next);
	E[1][1] = strtod(next, &next);
	E[1][2] = strtod(next, &next);
	E[2][0] = strtod(next, &next);
	E[2][1] = strtod(next, &next);
	E[2][2] = strtod(next, &next);

	return E;
}

CVECTOR<double> rcv_e(std::string msg)
{
	char rcv_err [CHAR_SIZE];
	CVECTOR<double> e(n);
	double dummy;
	char *next;

	memset(&rcv_err[0], 0, sizeof(rcv_err));
	for(int i=0; i<=msg.size(); i++) rcv_err[i] = msg[i];
	dummy = strtod(rcv_err, &next);
      for(int i=1; i<9; i++) dummy = strtod(next, &next);

	e[0] = strtod(next, &next);
	e[1] = strtod(next, &next);
	e[2] = strtod(next, &next);

	return e;
}
