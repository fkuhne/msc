/* controls.h */

#ifndef CONTROLS_H
#define CONTROLS_H

#include "mmath.h"

#define n 3
#define p 2

double vl;						// velocidade linear do robô.
double r;						// raio da trajetória do robô.
double va;						// velocidade angular em funcao da velocidade tangencial e o raio.
double T;						// periodo de amostragem de 100 ms.
double k_final;					// tempo total dividido pelo periodo de amostragem.

extern CVECTOR<double> ref_point(const CVECTOR<double> y, const CVECTOR<double> r_r)
{
	CVECTOR<double> y_r(n);

      // Ponto de referência com relação ao sistema móvel do robô:
	y_r[0] = y[0] + r_r[0] * cos(y[2]) - r_r[1] * sin(y[2]);
	y_r[1] = y[1] + r_r[0] * sin(y[2]) + r_r[1] * cos(y[2]);
	y_r[2] = y[2];
	
	return y_r;

}


extern CVECTOR<double> linearization(const CVECTOR<double> y, const CVECTOR<double> r_p, const CVECTOR <double> v)
{
	MATRIX<double> B(p,p);

      // Lei linearizante:
	B[0][0] = cos(y[2]); B[0][1] = -(r_p[0] * sin(y[2]) - r_p[1] * cos(y[2]));
	B[1][0] = sin(y[2]); B[1][1] =   r_p[0] * cos(y[2]) + r_p[1] * sin(y[2]);

	return B.inv() * v;
}


extern CVECTOR<double> control(const CVECTOR<double> y_ref, const CVECTOR<double> y_r, const MATRIX<double> K)
{
	CVECTOR<double> e(p);

      // Lei de controle linear com relação ao erro:
	e[0] = y_ref[0] - y_r[0];
	e[1] = y_ref[1] - y_r[1];

	return K * e;
}

#endif
