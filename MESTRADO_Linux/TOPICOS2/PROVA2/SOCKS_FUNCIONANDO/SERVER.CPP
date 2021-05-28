#include "mmath.h"
#include "ServerSocket.h"
#include <string.h>
#include <stdio.h>

#define PORT_ENC 30000					// endereço da porta do encoder.
#define PORT_GPS 31000					// endereço da porta do GPS.
#define PORT_BUS 32000					// endereço da porta da bússola.

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#define n 3						// ordem de sistema.
#define p 2						// ordem do vetor de entrada.

#define var_x 0.0005					// variância do processo em x.
#define var_y 0.0005					// variância do processo em y.
#define var_theta 0.0008				// variância do processo em theta.

CVECTOR<double> x(n);					// vetor de estados do processo.
CVECTOR<double> u(p);					// vetor de entradas de controle.
CVECTOR<double> w(n);					// vetor de ruído do processo.

CVECTOR<double> f(const CVECTOR<double> A,const CVECTOR<double> U,double T)
{
	// Calcula x[k+1] = f(x[k],u[k]).
	CVECTOR<double> B(n);
	B[0] = A[0] + U[0] * cos(A[2]) * T;
	B[1] = A[1] + U[0] * sin(A[2]) * T;
	B[2] = A[2] + U[1] * T;
	return B;
}

double normal(double m, double s)
{
	// retorna uma variável pseudo-aleatória com distribuição normal.
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
	std::string msg;				// mensagem recebida dos clientes.
	char buffer [200];				// char para enviar [x y theta].

	double vl = 0.3;				// velocidade linear do robô.
	double r = 2;					// raio da trajetória.
	double va = vl/r;				// velocidade angular do robô.
	double T = 0.1;					// período de amostragem.
	double k_final = (4*M_PI/va)/T;			// tempo final de simulação (8 completo).
	double k_meio = k_final/2;			// metade da trajetória (qdo inverte a va).

	x.zeros();

	ServerSocket server_enc (PORT_ENC);		// cria servidor.
	ServerSocket client_enc;			// cria cliente.
	server_enc.accept (client_enc);			// conecta servidor com cliente.

	ServerSocket server_gps (PORT_GPS);		// cria servidor.
	ServerSocket client_gps;			// cria cliente.
	server_gps.accept (client_gps);			// conecta servidor com cliente.

	ServerSocket server_bus (PORT_BUS);		// cria servidor.
	ServerSocket client_bus;			// cria cliente.
	server_bus.accept (client_bus);			// conecta servidor com cliente.

	std::cout << "\n--- Process running...\n";

	for (int k=0; k<k_final; k++)
	{
		if (k>=k_meio && va>0) va = -va;
		u[0] = vl;
		u[1] = va;

		w[0] = normal(0,sqrt(var_x));
		w[1] = normal(0,sqrt(var_y));
		w[2] = normal(0,sqrt(var_theta));

		x = f(x,u,T) + w;
		snprintf (buffer, sizeof(buffer), "%g %g %g %g %g", x[0], x[1], x[2], u[0], u[1]);

		client_enc >> msg;			// Aguarda requisição do encoder...
		client_enc << buffer;			// ... e envia o dado para ele.

		client_gps >> msg;			// Aguarda requisição do GPS...
		client_gps << buffer;			// ... e envia o dado para ele.

		client_bus >> msg;			// Aguarda requisição da bússola...
		client_bus << buffer;			// ... e envia o dado para ela.

		std::cout << " Step " << k << "\n";
	}

	client_enc >> msg;
	client_gps >> msg;
	client_bus >> msg;

	std::cout << "\n--- Process done!\n\n";
	
	return 0;
}

// ESTA É A ÚLTIMA LINHA.
