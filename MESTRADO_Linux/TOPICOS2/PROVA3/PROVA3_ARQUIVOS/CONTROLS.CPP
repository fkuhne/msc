/* controls.cpp */

#include "mmath.h"

#define n 3
#define p 2

MATRIX<double> R_O2C (const CVECTOR<double> q)
{
	MATRIX<double>R(n,n);

	// Matriz de transformação de coordenadas do sistema global para o sistema móvel do robô:
	R[0][0] = cos(q[2]);  R[0][1] = sin(q[2]); R[0][2] = 0;
	R[1][0] = -sin(q[2]); R[1][1] = cos(q[2]); R[1][2] = 0;
	R[2][0] = 0;          R[2][1] = 0;         R[2][2] = 1;

	return R;
}


CVECTOR<double> go_to_ref_point(const CVECTOR<double> q, const CVECTOR<double> r_r)
{
      // Move um ponto no centro de massa do robô (C) para um ponto de referência qualquer (r_r):
      return R_O2C(q).t() * r_r + q;
}


CVECTOR<double> linearization(const CVECTOR<double> q, const CVECTOR<double> r_r, const CVECTOR <double> v)
{
	MATRIX<double> B(p,p);

      // Lei linearizante:
	B[0][0] = cos(q[2]); B[0][1] = -(r_r[0] * sin(q[2]) + r_r[1] * cos(q[2]));
	B[1][0] = sin(q[2]); B[1][1] =   r_r[0] * cos(q[2]) - r_r[1] * sin(q[2]);

	return B.inv() * v;
}


CVECTOR<double> control(const CVECTOR<double> q_ref, const CVECTOR<double> q_r, const MATRIX<double> L)
{
	CVECTOR<double> e(p);

      // Lei de controle linear com relação ao erro:
	e[0] = q_ref[0] - q_r[0];
	e[1] = q_ref[1] - q_r[1];

	return L * e;
}
