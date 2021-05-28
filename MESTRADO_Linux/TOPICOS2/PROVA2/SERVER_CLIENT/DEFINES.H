/* defines.h */

/* Este header inclui todas as definições e declarações que são comuns aos nodos server, encoder, gps e bússola. */

#ifndef DEFINES_H
#define DEFINES_H

#include "normal.h"
#include "func.h"
#include "mmath.h"
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

//#define PORT_SIM2ENC 20000					// porta q comunica o encoder (cliente) com a simulacao (servidor).

#define n 3								// ordem do sistema.
#define p 2								// ordem da entrada.
                                                                         
#define sigma_x 0.0015						// variância do estado x.
#define sigma_y 0.0015						// variância do estado y.
#define sigma_theta 0.00075					// variância do estado theta.

#define CHAR_SIZE 2000
#define SEND_ERR_SIZE 12					// tamanho do vetor de erros a ser enviado.

CVECTOR<double> x(n);						// vetor de estados reais (recebido do servidor).
CVECTOR<double> u(p);						// vetor de entradas de controle.

CVECTOR<double> x_est_k(n);					// estado local estimado em (k|k).
CVECTOR<double> x_est_k1(n);					// estado global estimado em (k|k-1).

MATRIX<double> I(n,n);						// matriz identidade.
MATRIX<double> Q(n,n);						// matriz de covariâncias do processo.
MATRIX<double> F(n,n);						// jacobiano de f(x,u).
MATRIX<double> P_k(n,n);					// covariância global em (k|k).
MATRIX<double> P_k1(n,n);		      		// covariância global em (k|k-1).

#endif
