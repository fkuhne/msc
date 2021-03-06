/* control_defines.h */

#ifndef CONTROL_DEFINES_H
#define CONTROL_DEFINES_H

#include "func.h"
#include "controls.h"
#include <fstream.h>
#include <unistd.h> 							// necess�rio para o execlp

#define n 3									// ordem do sistema.
#define p 2									// ordem da entrada.
#define m 3									// ordem da sa�da.
#define m_c 2								// ordem do controle.
                                                                                    
#define sigma_x 0.0015							// vari�ncia do estado x.
#define sigma_y 0.0015							// vari�ncia do estado y.
#define sigma_theta 0.00075						// vari�ncia do estado theta.

#define sigma_out_x 0.01						// desvio padr�o da medida x.
#define sigma_out_y 0.01						// desvio padr�o da medida y.
#define sigma_out_theta 0.018						// desvio padr�o da medida theta.

#define x_ref_p 0.2							// ponto de refer�ncia em x com rela��o ao sistema m�vel do rob�.
#define y_ref_p 0								// ponto de refer�ncia em y com rela��o ao sistema m�vel do rob�.

#define k_x 3								// ganho da lei de controle linear.
#define k_y 3								// ganho da lei de controle linear.

CVECTOR<double> x(n);							// vetor de estados reais.

CVECTOR<double> u(p);							// vetor de entradas de controle.
CVECTOR<double> y(m);							// sa�da com rela��o ao centro.
CVECTOR<double> x_ref(n);						// trajet�ria de refer�ncia.
CVECTOR<double> u_ref(p);						// vetor de entradas de controle de refer�ncia.
CVECTOR<double> y_ref(m);						// sa�da de refer�ncia.

CVECTOR<double> r_r(m);							// ponto de refer�ncia.
CVECTOR<double> y_r(m);							// sa�da com rela��o ao ponto de refer�ncia.
CVECTOR<double> v(m_c);							// entrada de controle linear.

MATRIX<double> H(m,m);							// matriz de observa��o.
MATRIX<double> K(m_c,m_c);						// matriz de ganhos do controle linear.

#endif
