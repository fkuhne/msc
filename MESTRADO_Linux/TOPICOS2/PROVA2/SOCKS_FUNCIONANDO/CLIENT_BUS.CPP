#include "mmath.h"
#include "ClientSocket.h"
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>

#define PORT_BUS2SERV 32000			// porta q comunica a bússola com o servidor.
#define PORT_BUS2GPS 42000			// porta q comunica a bússola com o GPS.

#define n 3					// ordem do sistema
#define p 2					// ordem da entrada
#define m_bus 1					// ordem da saída da bússola

#define var_x 0.00005				// variância do estado x.
#define var_y 0.00005				// variância do estado y.
#define var_theta 0.00008			// variância do estado theta.

#define var_bus_theta 0.0175			// variância da medida x_bus.

CVECTOR<double> x(n);				// vetor de estados reais (recebido do servidor).
CVECTOR<double> u(p);				// vetor de entradas de controle.

CVECTOR<double> x_bus_est_local_k(n);		// estado local estimado em (k|k).
CVECTOR<double> x_bus_est_global_k(n);		// estado global estimado em (k|k).
CVECTOR<double> x_bus_est_global_k1(n);		// estado global estimado em (k|k-1).
CVECTOR<double> e_bus(n);			// erro de estimação.
CVECTOR<double> e_enc(n);			// erro de estimação.
CVECTOR<double> e_gps(n);			// erro de estimação.
CVECTOR<double> y_bus(m_bus);			// vetor de saída.
CVECTOR<double> v_bus(m_bus);			// ruído de medida.
CVECTOR<double> i_bus(m_bus);			// vetor de inovação.
MATRIX<double> I(n,n);				// matriz identidade.
MATRIX<double> Q(n,n);				// matriz de covariâncias do processo.
MATRIX<double> F_bus(n,n);			// jacobiano de f(x,u).
MATRIX<double> P_bus_local_k(n,n);		// covariância local em (k|k).
MATRIX<double> P_bus_global_k(n,n);		// covariância global em (k|k).
MATRIX<double> P_bus_global_k1(n,n);		// covariância global em (k|k-1).
MATRIX<double> E_bus(n,n);			// covariância do erro.
MATRIX<double> E_enc(n,n);			// covariância do erro.
MATRIX<double> E_gps(n,n);			// covariância do erro.
MATRIX<double> K_bus(n,m_bus);			// ganho do EKF.
MATRIX<double> H_bus(m_bus,n);			// Jacobiano de h(x).
MATRIX<double> R_bus(m_bus,m_bus);		// variâncias da medida.


CVECTOR<double> f(const CVECTOR<double> A,const CVECTOR<double> U,double T)
{
	// Calcula x[k+1] = f(x[k],u[k]).
	CVECTOR<double> B(n);
	B[0] = A[0] + U[0] * cos(A[2]) * T;
	B[1] = A[1] + U[0] * sin(A[2]) * T;
	B[2] = A[2] + U[1] * T;
	return B;
}


MATRIX<double> jacob_F(const CVECTOR<double> A, const CVECTOR<double> U, double T)
{
	// Calcula o jacobiano de f(^x[k]):
	MATRIX<double> F(n,n);
	F.eye();
	F[0][2] = -sin(A[2]) * U[0] * T;
	F[1][2] = cos(A[2]) * U[0] * T;
	return F;
}


double normal (double m, double s)
{
	// Retorna uma variável pseudo-aleatória com distribuição normal:
	static int iset = 0;
	static double gset;
	double fac,r,v1,v2;

	if (!iset)
	{
		do
		{
			v1 = 2.0 * rand()/(RAND_MAX+1.0)-1.0;
			v2 = 2.0 * rand()/(RAND_MAX+1.0)-1.0;
			r = v1*v1 + v2*v2;
		} while (r>=1.0);

		fac = sqrt(-2.0*log(r)/r);
		gset = v1 * fac;
		iset = 1;
		return v2 * fac * s + m;
	}
	else
	{
		iset=0;
		return gset * s + m;
	}
}


int main (void)
{
	std::string msg;
	char carac [100];				// vetor auxiliar para separar os dados recebidos do servidor.
	char rcv_err[2000];				// vetor para receber os erros 'e' e 'E' dos outros nodos.
	char snd_err[2000];				// vetor para enviar os erros 'e' e 'E' dos outros nodos.
	char *next;					// ponteiro auxiliar.
	double T = 0.1;					// período de amostragem.

	// Cria arquivo de nome "xxx_bus", onde serão inseridos os dados:
	ofstream arq("./xxx_bus");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICAÇÃO:
	//
	// Cliente para comunicar com o servidor:
	ClientSocket client_bus2serv ("localhost", PORT_BUS2SERV);	// cria cliente.

	// Cliente para comunicar com o GPS:
	ClientSocket client_gps2bus ("localhost", PORT_BUS2GPS);	// cria cliente.
	//
	// -------------------------------------------------------

	// Inicializa variáveis:
	x.zeros();
	x_bus_est_global_k1.zeros();
	H_bus.zeros(); H_bus[0][2]=1;
	I.eye();
	R_bus[0][0] = var_bus_theta;
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_bus_global_k1.eye(); P_bus_global_k1 = P_bus_global_k1*1000;

	std::cout << "\n--- Client connected...\n";

	while (true)
	{
		// Comunica com o servidor para receber os valores de [x y theta] do processo:
		client_bus2serv << "anything";				// requisita dado ao servidor.
		client_bus2serv >> msg;					// recebe dado do servidor.

		for(int i=0; i<=msg.size(); i++) carac[i] = msg[i];	// estas linhas transformam
		x[0] = strtod(carac, &next);				// o dado recebido (x,y,theta,u1,u2)
		x[1] = strtod(next, &next);				// em 5 valores <double> que serão
		x[2] = strtod(next, &next);				// alocados em um vetor [x;y;theta]
		u[0] = strtod(next, &next);				// e em um vetor [u1 u2].
		u[1] = strtod(next, &next);

		v_bus[0] = normal(0,sqrt(var_bus_theta));
		y_bus = H_bus * x + v_bus;				// expressão de observação.

		// ------------------------------------------------------------------
		// Início do algoritmo do Filtro de Kalman Extendido Descentralizado:
		//
		//passo 1 -    K(k) = P(k|k-1) * Ht(k) *inv(H(k) * P(k|k-1) * Ht(k) + R(k))
		K_bus = P_bus_global_k1 * H_bus.t() * (H_bus * P_bus_global_k1 * H_bus.t() + R_bus).inv();

		i_bus[0] = y_bus[0] - x_bus_est_global_k1[2];

		//passo 2 -    ~x(k|k) = ^x(k|k-1) + K(k) * (y(k) - h(^x(k|k-1), u(k)))
		x_bus_est_local_k = x_bus_est_global_k1 + K_bus * i_bus;

		//passo 3 -    ~P(k|k) = (I - K(k) * H(k)) * P(k|k-1)
		P_bus_local_k = (I - K_bus * H_bus) * P_bus_global_k1;

		//passo 4 -    e(k) = inv(~P(k|k)) * ~x(k|k) - inv(P(k|k-1)) * ^x(k|k-1)
		e_bus = P_bus_local_k.inv() * x_bus_est_local_k - P_bus_global_k1.inv() * x_bus_est_global_k1;

		//passo 5 -    E(k) = inv(~P(k|k)) - inv(P(k|k-1))
		E_bus = P_bus_local_k.inv() - P_bus_global_k1.inv();


		// TENTATIVA: zerar os vetores de erro rcv_err e snd_err.


		// Grava os erros na variável snd_err:
		snprintf (snd_err, sizeof(snd_err), "%g %g %g %g %g %g %g %g %g %g %g %g", \
						      E_bus[0][0],E_bus[0][1],E_bus[0][2], \
						      E_bus[1][0],E_bus[1][1],E_bus[1][2], \
						      E_bus[2][0],E_bus[2][1],E_bus[2][2], \
						      e_bus[0],   e_bus[1],   e_bus[2]);

		client_gps2bus << snd_err;				// envia os erros para o GPS.

		client_gps2bus >> msg;					// recebe os erros do encoder e do GPS.

		for(int i=0; i<=msg.size(); i++) rcv_err[i] = msg[i];
		E_enc[0][0] = strtod(rcv_err, &next);
		E_enc[0][1] = strtod(next, &next);
		E_enc[0][2] = strtod(next, &next);
		E_enc[1][0] = strtod(next, &next);
		E_enc[1][1] = strtod(next, &next);
		E_enc[1][2] = strtod(next, &next);
		E_enc[2][0] = strtod(next, &next);
		E_enc[2][1] = strtod(next, &next);
		E_enc[2][2] = strtod(next, &next);
		e_enc[0] = strtod(next, &next);
		e_enc[1] = strtod(next, &next);
		e_enc[2] = strtod(next, &next);

		E_gps[0][0] = strtod(next, &next);
		E_gps[0][1] = strtod(next, &next);
		E_gps[0][2] = strtod(next, &next);
		E_gps[1][0] = strtod(next, &next);
		E_gps[1][1] = strtod(next, &next);
		E_gps[1][2] = strtod(next, &next);
		E_gps[2][0] = strtod(next, &next);
		E_gps[2][1] = strtod(next, &next);
		E_gps[2][2] = strtod(next, &next);
		e_gps[0] = strtod(next, &next);
		e_gps[1] = strtod(next, &next);
		e_gps[2] = strtod(next, &next);


		// Bom, continuando com o EKF...

		//passo 6 -    P(k|k) = inv(inv(P(k|k-1)) + E_enc + E_gps + E_bus)
		P_bus_global_k = (P_bus_global_k1.inv() + E_enc + E_gps + E_bus).inv();

		//passo 7 -    ^x(k|k) = P(k|k) * ( inv(P(k|k-1)) * ^x(k|k-1) + e_enc + e_gps + e_bus )
		x_bus_est_global_k = P_bus_global_k * (P_bus_global_k1.inv() * x_bus_est_global_k1 + e_enc + e_gps + e_bus);

		//passo 8 -    ^x(k+1|k) = f(^x(k|k), u(k))
		x_bus_est_global_k1 = f(x_bus_est_global_k,u,T);

		F_bus = jacob_F(x_bus_est_global_k,u,T);

		//passo 9 -    P(k+1|k) = F(k) * P(k|k) * Ft(k) + Q(k)
		P_bus_global_k1 = F_bus * P_bus_global_k * F_bus.t() + Q;
		//
		// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
		// ---------------------------------------------------------------


		arq
			<< x_bus_est_global_k[0] << "\t"        // 1
			<< x_bus_est_global_k[1] << "\t"        // 2
			<< x_bus_est_global_k[2] << "\t"        // 3

			<< P_bus_global_k[0][0] << "\t"         // 4
			<< P_bus_global_k[1][1] << "\t"         // 5
			<< P_bus_global_k[2][2] << "\t"         // 6

			<< x[0] << "\t"                         // 7
			<< x[1] << "\t"                         // 8
			<< x[2] << "\t"                         // 9

			<< "\n";
	}

	arq.close();

	std::cout << "\n--- Client done!\n";
		
	client_bus2serv << "anything";

	return 0;
}
