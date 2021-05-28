/* control_defines.h */

#ifndef CONTROL_DEFINES_H
#define CONTROL_DEFINES_H

#include "func.h"
#include "controls.h"
#include <fstream.h>
#include <unistd.h> 							// necessário para o execlp

#define n 3									// ordem do sistema.
#define p 2									// ordem da entrada.
#define m 3									// ordem da saída.
#define m_c 2								// ordem do controle.
                                                                                    
#define sigma_x 0.0015							// variância do estado x.
#define sigma_y 0.0015							// variância do estado y.
#define sigma_theta 0.00075						// variância do estado theta.

#define sigma_out_x 0.01						// desvio padrão da medida x.
#define sigma_out_y 0.01						// desvio padrão da medida y.
#define sigma_out_theta 0.018						// desvio padrão da medida theta.

#define x_ref_p 0.2							// ponto de referência em x com relação ao sistema móvel do robô.
#define y_ref_p 0								// ponto de referência em y com relação ao sistema móvel do robô.

#define k_x 3								// ganho da lei de controle linear.
#define k_y 3								// ganho da lei de controle linear.

CVECTOR<double> x(n);							// vetor de estados reais.

CVECTOR<double> u(p);							// vetor de entradas de controle.
CVECTOR<double> y(m);							// saída com relação ao centro.
CVECTOR<double> x_ref(n);						// trajetória de referência.
CVECTOR<double> u_ref(p);						// vetor de entradas de controle de referência.
CVECTOR<double> y_ref(m);						// saída de referência.

CVECTOR<double> r_r(m);							// ponto de referência.
CVECTOR<double> y_r(m);							// saída com relação ao ponto de referência.
CVECTOR<double> v(m_c);							// entrada de controle linear.

MATRIX<double> H(m,m);							// matriz de observação.
MATRIX<double> K(m_c,m_c);						// matriz de ganhos do controle linear.

#endif
