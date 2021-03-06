/* defines.h */

/* Este header inclui todas as defini��es e declara��es que s�o comuns aos nodos server, encoder, gps e b�ssola. */

#ifndef DEFINES_H
#define DEFINES_H

#include "normal.h"
#include "func.h"
#include "mmath.h"
#include "ServerSocket.h"
#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define PORT_SIM2ENC 20000					// porta q comunica o encoder (cliente) com a simulacao (servidor).
#define PORT_SIM2GPS 21000					// porta q comunica o GPS (cliente) com a simulacao (servidor).
#define PORT_SIM2BUS 22000					// porta q comunica a bussola (cliente) com a simulacao (servidor).

#define PORT_ENC2GPS 23000					// porta q comunica o encoder (servidor) com o GPS (cliente).
#define PORT_ENC2BUS 24000					// porta q comunica o encoder (servidor) com a bussola (cliente).
#define PORT_GPS2ENC 25000					// porta q comunica o GPS (servidor) com o encoder (cliente).
#define PORT_GPS2BUS 26000					// porta q comunica o GPS (servidor) com a bussola (cliente).
#define PORT_BUS2ENC 27000					// porta q comunica a bussola (servidor) com o encoder (cliente).
#define PORT_BUS2GPS 28000					// porta q comunica a bussola (servidor) com o GPS (cliente).

#define n 3								// ordem do sistema.
#define p 2								// ordem da entrada.
                                                                         
#define sigma_x 0.0015						// vari�ncia do estado x.
#define sigma_y 0.0015						// vari�ncia do estado y.
#define sigma_theta 0.0018					// vari�ncia do estado theta.
#define var_x (sigma_x*sigma_x)
#define var_y (sigma_y*sigma_y)
#define var_theta (sigma_theta*sigma_theta)

#define CHAR_SIZE 2000
#define SEND_ERR_SIZE 12					// tamanho do vetor de erros a ser enviado.

CVECTOR<double> x(n);						// vetor de estados reais (recebido do servidor).
CVECTOR<double> u(p);						// vetor de entradas de controle.
CVECTOR<double> w(n);						// vetor dos ru�dos do processo.

CVECTOR<double> x_est_local_k(n);				// estado local estimado em (k|k).
CVECTOR<double> x_est_global_k(n);				// estado global estimado em (k|k).
CVECTOR<double> x_est_global_k1(n);				// estado global estimado em (k|k-1).

MATRIX<double> I(n,n);						// matriz identidade.
MATRIX<double> Q(n,n);						// matriz de covari�ncias do processo.
MATRIX<double> F(n,n);						// jacobiano de f(x,u).
MATRIX<double> P_local_k(n,n);				// covari�ncia local em (k|k).
MATRIX<double> P_global_k(n,n);				// covari�ncia global em (k|k).
MATRIX<double> P_global_k1(n,n);	      		// covari�ncia global em (k|k-1).

CVECTOR<double> e_bus(n);					// erro de estima��o.
CVECTOR<double> e_enc(n);					// erro de estima��o.
CVECTOR<double> e_gps(n);					// erro de estima��o.
MATRIX<double> E_bus(n,n);					// covari�ncia do erro.
MATRIX<double> E_enc(n,n);					// covari�ncia do erro.
MATRIX<double> E_gps(n,n);					// covari�ncia do erro.

#endif
