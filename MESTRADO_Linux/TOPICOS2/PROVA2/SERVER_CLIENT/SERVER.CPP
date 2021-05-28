/* server.cpp */

#include "defines.h"

CVECTOR<double> w(n);

int main (void)
{
	std::string msg;						// mensagem recebida dos clientes.
	char buffer [400];					// char para enviar [x y theta].

	double vl = 0.3;						// velocidade linear do robô.
	double r = 2;						// raio da trajetória.
	double va = vl/r;						// velocidade angular do robô.
	double T = 0.1;						// período de amostragem.
	double k_final = (4*M_PI/va)/T;			// tempo final de simulação (8 completo).
	double k_meio = k_final/2;				// metade da trajetória (qdo inverte a va).

      // --------------------------------------------
      // MECANISMOS DE COMUNICAÇÃO:
      //
      // Servidor para se comunicar com o encoder:
	ServerSocket server_enc (PORT_SIM2ENC);
	ServerSocket client_enc;
	cout << "Cria servidor server_enc... "; cout.flush();
	server_enc.accept (client_enc);
	cout << "conectou servidor server_enc    PASSO 1.2\n";
	std::cout << "\n--- Processo rodando...\n";
	//
      // --------------------------------------------

	x.zeros();

	for (int k=0; k<k_final; k++)
	{
		if (k>=k_meio && va>0) va = -va;
		u[0] = vl;
		u[1] = va;

		w[0] = normal(0,sigma_x);
		w[1] = normal(0,sigma_y);
		w[2] = normal(0,sigma_theta);
		x = f(x,u,T) + w;
		
		snprintf (buffer, sizeof(buffer), "%g %g %g %g %g", x[0], x[1], x[2], u[0], u[1]);
		client_enc >> msg;				// Aguarda requisição do encoder...
		client_enc << buffer;				// ... e envia o dado para ele.

		std::cout << " Step " << k << "\n";
	}
	std::cout << "\n--- Processo finalizado!\n\n" << '\x07';
	return 0;
}

/* server.cpp -- ESTA É A ÚLTIMA LINHA */
