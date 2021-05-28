/* control_defines.h */

#include "mmath.h"
#include "ServerSocket.h"
#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define CHAR_SIZE 2000

//
// DEFINIÇÕES DO MODELO:
//
#define n 3								// ordem do sistema.
#define p 2								// ordem da entrada.
#define m 3								// ordem da saida.

#define sigma_x 0.0015*50						// desvio padrão do estado x.
#define sigma_y 0.0015*50						// desvio padrão do estado y.
#define sigma_theta 0.0018*50				// desvio padrão do estado theta.
#define var_x (sigma_x * sigma_x)
#define var_y (sigma_y * sigma_y)
#define var_theta (sigma_theta * sigma_theta)

CVECTOR<double> q(n);						// vetor de estados reais.
CVECTOR<double> u(p);						// vetor de entradas de controle.
CVECTOR<double> w(n);						// vetor de ruidos do processo.
MATRIX<double> F(n,n);						// jacobiano de f(x,u).
MATRIX<double> H(m,n);						// matriz de observação.

//
// DEFINIÇÕES DO CONTROLE:
//
#define m_c 2							// ordem do controle.

#define x_p_ref 0.4						// ponto de referência R em x.
#define y_p_ref 0							// ponto de referência R em y.

#define l_x 3							// ganho da lei de controle linear.
#define l_y 3							// ganho da lei de controle linear.

CVECTOR<double> q_ref(n);					// trajetória de referência.
CVECTOR<double> q_r_ref(m);					// saída de referência.
CVECTOR<double> q_est(m);					// posição estimada com relação ao centro do robô.
CVECTOR<double> q_r_est(m);					// posição estimada com relação ao ponto de referência.
CVECTOR<double> r_r(m);						// ponto de referência.
CVECTOR<double> v(m_c);						// entrada de controle linear.
MATRIX<double> L(m_c,m_c);					// matriz de ganhos do controle linear.
CVECTOR<double> u_ref(p);					// vetor de entradas de controle de referência.

//
// DEFINIÇÕES DO FILTRO DE KALMAN:
//
CVECTOR<double> q_est_local_k(n);				// estado local estimado em (k|k).
CVECTOR<double> q_est_global_k(n);				// estado global estimado em (k|k).
CVECTOR<double> q_est_global_k1(n);				// estado global estimado em (k|k-1).
CVECTOR<double> e_bus(n);					// erro de estimação.
CVECTOR<double> e_enc(n);					// erro de estimação.
CVECTOR<double> e_gps(n);					// erro de estimação.
MATRIX<double> E_bus(n,n);					// covariância do erro.
MATRIX<double> E_enc(n,n);					// covariância do erro.
MATRIX<double> E_gps(n,n);					// covariância do erro.
MATRIX<double> I(n,n);						// identidade.
MATRIX<double> K(n,m);						// ganho do filtro de Kalman.
MATRIX<double> Q(n,n);						// matriz de covariâncias do processo.
MATRIX<double> P_local_k(n,n);				// covariância local em (k|k).
MATRIX<double> P_global_k(n,n);				// covariância global em (k|k).
MATRIX<double> P_global_k1(n,n);				// covariância global em (k|k-1).

//
// DEFINIÇÕES DOS SOCKETS:
//
#define PORT_SIM2ENC 20000					// porta q comunica o encoder (cliente) com a simulacao (servidor).
#define PORT_SIM2GPS 21000					// porta q comunica o GPS (cliente) com a simulacao (servidor).
#define PORT_SIM2BUS 22000					// porta q comunica a bussola (cliente) com a simulacao (servidor).
#define PORT_ENC2GPS 23000					// porta q comunica o encoder (servidor) com o GPS (cliente).
#define PORT_ENC2BUS 24000					// porta q comunica o encoder (servidor) com a bussola (cliente).
#define PORT_GPS2ENC 25000					// porta q comunica o GPS (servidor) com o encoder (cliente).
#define PORT_GPS2BUS 26000					// porta q comunica o GPS (servidor) com a bussola (cliente).
#define PORT_BUS2ENC 27000					// porta q comunica a bussola (servidor) com o encoder (cliente).
#define PORT_BUS2GPS 28000					// porta q comunica a bussola (servidor) com o GPS (cliente).
