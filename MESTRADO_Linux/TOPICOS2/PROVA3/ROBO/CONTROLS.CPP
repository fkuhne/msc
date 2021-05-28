/* controls.cpp */

#include "mmath.h"

#define n 3
#define p 2

MATRIX<double> R_OC (const CVECTOR<double> y)
{
	MATRIX<double>R(n,n);

	// Matriz de transformação de coordenadas do sistema global para o sistema móvel do robô:
	R[0][0] = cos(y[2]);  R[0][1] = sin(y[2]); R[0][2] = 0;
	R[1][0] = -sin(y[2]); R[1][1] = cos(y[2]); R[1][2] = 0;
	R[2][0] = 0;          R[2][1] = 0;         R[2][2] = 1;

	return R;
}


CVECTOR<double> go_to_ref_point(const CVECTOR<double> y, const CVECTOR<double> r_r)
{
      // Move um ponto no centro de massa do robô (C) para um ponto de referência qualquer (r_r):
      return R_OC(y).t() * r_r + y;
}


CVECTOR<double> go_to_center_point(const CVECTOR<double> y_ref, const CVECTOR<double> r_r)
{
      // Move um ponto de referência (r_r) para o centro de massa do robô (C):
      return y_ref - R_OC(y_ref).t() * r_r;
}


CVECTOR<double> linearization(const CVECTOR<double> y, const CVECTOR<double> r_p, const CVECTOR <double> v)
{
	MATRIX<double> B(p,p);

      // Lei linearizante:
	B[0][0] = cos(y[2]); B[0][1] = -(r_p[0] * sin(y[2]) - r_p[1] * cos(y[2]));
	B[1][0] = sin(y[2]); B[1][1] =   r_p[0] * cos(y[2]) + r_p[1] * sin(y[2]);

	return B.inv() * v;
}


CVECTOR<double> control(const CVECTOR<double> y_ref, const CVECTOR<double> y_r, const MATRIX<double> K)
{
	CVECTOR<double> e(p);

      // Lei de controle linear com relação ao erro:
	e[0] = y_ref[0] - y_r[0];
	e[1] = y_ref[1] - y_r[1];

	return K * e;
}
