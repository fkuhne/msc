#include "mmath.h"
#include "ClientSocket.h"
#include "ServerSocket.h"
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>

#define PORT_GPS2SERV 31000			// porta q comunica o GPS(cliente) com o processo (servidor).
#define PORT_GPS2ENC 40000			// porta q comunica o GPS(cliente) com o encoder(servidor).
#define PORT_GPS2BUS 42000			// porta q comunica o GPS(servidor) com a bússola(cliente).

#define n 3					// ordem do sistema.
#define p 2					// ordem da entrada.
#define m_gps 2					// ordem da saída do GPS.

#define var_x 0.00005				// variância do estado x.
#define var_y 0.00005				// variância do estado y.
#define var_theta 0.00008			// variância do estado theta.

#define var_gps_x 0.01				// variância da medida x_gps.
#define var_gps_y 0.01				// variância da medida y_gps.

CVECTOR<double> x(n);				// vetor de estados reais (recebido do servidor).
CVECTOR<double> u(p);					// vetor de entradas de controle.

CVECTOR<double> x_gps_est_local_k(n);		// estado local estimado em (k|k).
CVECTOR<double> x_gps_est_global_k(n);		// estado global estimado em (k|k).
CVECTOR<double> x_gps_est_global_k1(n);		// estado global estimado em (k|k-1).
CVECTOR<double> e_gps(n);			// erro de estimação.
CVECTOR<double> e_enc(n);			// erro de estimação.
CVECTOR<double> e_bus(n);			// erro de estimação.
CVECTOR<double> y_gps(m_gps);			// vetor de saída.
CVECTOR<double> v_gps(m_gps);			// ruído de medida.
CVECTOR<double> i_gps(m_gps);			// vetor de inovação.
MATRIX<double> I(n,n);				// matriz identidade.
MATRIX<double> Q(n,n);				// matriz de covariâncias do processo.
MATRIX<double> F_gps(n,n);			// jacobiano de f(x,u).
MATRIX<double> P_gps_local_k(n,n);		// covariância local em (k|k).
MATRIX<double> P_gps_global_k(n,n);		// covariância global em (k|k).
MATRIX<double> P_gps_global_k1(n,n);		// covariância global em (k|k-1).
MATRIX<double> E_gps(n,n);			// covariância do erro.
MATRIX<double> E_enc(n,n);			// covariância do erro.
MATRIX<double> E_bus(n,n);			// covariância do erro.
MATRIX<double> K_gps(n,m_gps);			// ganho do EKF.
MATRIX<double> H_gps(m_gps,n);			// jacobiano de h(x).
MATRIX<double> R_gps(m_gps,m_gps);		// variâncias da medida.


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
	// Calcula o jacobiano de f(^x[k|k]):
	MATRIX<double> F(n,n);

	F.eye();
	F[0][2] = -sin(A[2]) * U[0] * T;
	F[1][2] = cos(A[2]) * U[0] * T;

	return F;
}


double normal (double m, double s)
{
	// Retorna uma variável pseudo-aleatória com distribuição normal
	static int iset = 0;
	static double gset;
	double fac,
	       r,
	       v1,
	       v2;

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
	char carac [100];
	char rcv_err[2000];				// vetor para receber os erros 'e' e 'E' dos outros nodos
	char snd_err[2000];				// vetor para enviar os erros 'e' e 'E' dos outros nodos
	char *next;
	double T = 0.1;					// período de amostragem.

	// Cria arquivo de nome "xxx_gps", onde serão inseridos os dados:
	ofstream arq("./xxx_gps");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICAÇÃO:
	//
	// Cliente para comunicar com o servidor:
	ClientSocket client_gps2serv ("localhost", PORT_GPS2SERV);	// cria cliente.

	// Cliente para comunicar com o encoder:
	ClientSocket client_enc2gps ("localhost", PORT_GPS2ENC);	// cria cliente.

	// Servidor para comunicar com a bússola:
	ServerSocket server_gps2bus (PORT_GPS2BUS);	// cria servidor.
	ServerSocket client_bus2gps;			// cria cliente.
	server_gps2bus.accept (client_bus2gps);		// conecta servidor com cliente.
	//
	// -------------------------------------------------------

	// Inicializa variáveis:
	x.zeros();
	x_gps_est_global_k1.zeros();
	H_gps.zeros(); H_gps[0][0]=1; H_gps[1][1]=1;
	I.eye();
	R_gps.zeros(); R_gps[0][0] = var_gps_x; R_gps[1][1] = var_gps_y;
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_gps_global_k1.eye(); P_gps_global_k1 = P_gps_global_k1*1000;

	std::cout << "\n--- Client connected...\n";

	while (true)
	{
		// Comunica com o servidor para receber os valores de [x y theta] do processo:
		client_gps2serv << "anything";				// requisita dado ao servidor.
		client_gps2serv >> msg;					// recebe dado do servidor.

		for(int i=0; i<=msg.size(); i++) carac[i] = msg[i];		// estas linhas transformam
		x[0] = strtod(carac, &next);					// o dado recebido (x,y,theta,u1,u2)
		x[1] = strtod(next, &next);                                     // em 5 valores <double> que serão
		x[2] = strtod(next, &next);					// alocados em um vetor [x;y;theta]
		u[0] = strtod(next, &next);					// e em um vetor [u1 u2].
		u[1] = strtod(next, &next);

		v_gps[0] = normal(0,sqrt(var_gps_x));
		v_gps[1] = normal(0,sqrt(var_gps_y));
		y_gps = H_gps * x + v_gps;				// expressão de observação.


		// ------------------------------------------------------------------
		// Início do algoritmo do Filtro de Kalman Extendido Descentralizado:
		//
		//passo 1 -    K(k) = P(k|k-1) * Ht(k) *inv(H(k) * P(k|k-1) * Ht(k) + R(k))
		K_gps = P_gps_global_k1 * H_gps.t() * (H_gps * P_gps_global_k1 * H_gps.t() + R_gps).inv();

		i_gps[0] = y_gps[0] - x_gps_est_global_k1[0]; i_gps[1] = y_gps[1] - x_gps_est_global_k1[1];

		//passo 2 -    ~x(k|k) = ^x(k|k-1) + K(k) * (y(k) - h(^x(k|k-1), u(k)))
		x_gps_est_local_k = x_gps_est_global_k1 + K_gps * i_gps;

		//passo 3 -    ~P(k|k) = (I - K(k) * H(k)) * P(k|k-1)
		P_gps_local_k = (I - K_gps * H_gps) * P_gps_global_k1;

		//passo 4 -    e(k) = inv(~P(k|k)) * ~x(k|k) - inv(P(k|k-1)) * ^x(k|k-1)
		e_gps = P_gps_local_k.inv() * x_gps_est_local_k - P_gps_global_k1.inv() * x_gps_est_global_k1;

		//passo 5 -    E(k) = inv(~P(k|k)) - inv(P(k|k-1))
		E_gps = P_gps_local_k.inv() - P_gps_global_k1.inv();


		// TENTATIVA: zerar os vetores de erro rcv_err e snd_err.


		client_bus2gps >> msg;				// recebe erros da bússola.

		for(int i=0; i<=msg.size(); i++) rcv_err[i] = msg[i];
		E_bus[0][0] = strtod(rcv_err, &next);
		E_bus[0][1] = strtod(next, &next);
		E_bus[0][2] = strtod(next, &next);
		E_bus[1][0] = strtod(next, &next);
		E_bus[1][1] = strtod(next, &next);
		E_bus[1][2] = strtod(next, &next);
		E_bus[2][0] = strtod(next, &next);
		E_bus[2][1] = strtod(next, &next);
		E_bus[2][2] = strtod(next, &next);
		e_bus[0] = strtod(next, &next);
		e_bus[1] = strtod(next, &next);
		e_bus[2] = strtod(next, &next);

		// Grava os erros na variável snd_err:
		// snd_err é um vetorzão de (size(E)+size(e))*2=24 elementos separados por espaços.
		snprintf (snd_err, sizeof(snd_err), "%g %g %g %g %g %g %g %g %g %g %g %g  \
						     %g %g %g %g %g %g %g %g %g %g %g %g",\
					             E_gps[0][0],E_gps[0][1],E_gps[0][2], \
						     E_gps[1][0],E_gps[1][1],E_gps[1][2], \
						     E_gps[2][0],E_gps[2][1],E_gps[2][2], \
						     e_gps[0],   e_gps[1],   e_gps[2],    \
						     E_bus[0][0],E_bus[0][1],E_bus[0][2], \
						     E_bus[1][0],E_bus[1][1],E_bus[1][2], \
						     E_bus[2][0],E_bus[2][1],E_bus[2][2], \
						     e_bus[0],   e_bus[1],   e_bus[2]);

		client_enc2gps << snd_err;			// envia os erros para o encoder.


		client_enc2gps >> msg;				// recebe os erros do encoder.

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

		// Grava os erros na variável snd_err:
		// snd_err é um vetorzão de (size(E)+size(e))*2=24 elementos separados por espaços.
		snprintf (snd_err, sizeof(snd_err), "%g %g %g %g %g %g %g %g %g %g %g %g  \
						     %g %g %g %g %g %g %g %g %g %g %g %g",\
					             E_enc[0][0],E_enc[0][1],E_enc[0][2], \
						     E_enc[1][0],E_enc[1][1],E_enc[1][2], \
						     E_enc[2][0],E_enc[2][1],E_enc[2][2], \
						     e_enc[0],   e_enc[1],   e_enc[2],    \
						     E_gps[0][0],E_gps[0][1],E_gps[0][2], \
						     E_gps[1][0],E_gps[1][1],E_gps[1][2], \
						     E_gps[2][0],E_gps[2][1],E_gps[2][2], \
						     e_gps[0],   e_gps[1],   e_gps[2]);


		client_bus2gps << snd_err;


		// Bom, continuando com o EKF...

		//passo 6 -    P(k|k) = inv(inv(P(k|k-1)) + E_enc + E_gps + E_bus)
		P_gps_global_k = (P_gps_global_k1.inv() + E_gps + E_enc + E_bus).inv();

		//passo 7 -    ^x(k|k) = P(k|k) * ( inv(P(k|k-1)) * ^x(k|k-1) + e_enc + e_gps + e_bus )
		x_gps_est_global_k = P_gps_global_k * (P_gps_global_k1.inv() * x_gps_est_global_k1 + e_gps + e_enc + e_bus);

		//passo 8 -    ^x(k+1|k) = f(^x(k|k), u(k))
		x_gps_est_global_k1 = f(x_gps_est_global_k,u,T);

		F_gps = jacob_F(x_gps_est_global_k,u,T);

		//passo 9 -    P(k+1|k) = F(k) * P(k|k) * Ft(k) + Q(k)
		P_gps_global_k1 = F_gps * P_gps_global_k * F_gps.t() + Q;
		//
                // Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
		// ---------------------------------------------------------------


		arq
			<< x_gps_est_global_k[0] << "\t"        // 1
			<< x_gps_est_global_k[1] << "\t"        // 2
			<< x_gps_est_global_k[2] << "\t"        // 3

			<< P_gps_global_k[0][0] << "\t"         // 4
			<< P_gps_global_k[1][1] << "\t"         // 5
			<< P_gps_global_k[2][2] << "\t"         // 6

			<< x[0] << "\t"                         // 7
			<< x[1] << "\t"                         // 8
			<< x[2] << "\t"                         // 9

			<< "\n";
	}

	arq.close();
	
	std::cout << "\n--- Client done!\n";
	
	client_gps2serv << "anything";

	return 0;
}
