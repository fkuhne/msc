/* simulador.cpp */

#include "defines.h"

CVECTOR<double> w(n);

int main (void)
{
	std::string msg;						// mensagem recebida dos clientes.
	char buffer [200];					// char para enviar [x y theta].

	double vl = 0.3;						// velocidade linear do robô.
	double r = 2;						// raio da trajetória.
	double va = vl/r;						// velocidade angular do robô.
	double T = 0.1;						// período de amostragem.
	double k_final = (4*M_PI/va)/T;			// tempo final de simulação (8 completo).
//	double k_final = 1000.;

      // --------------------------------------------
      // MECANISMOS DE COMUNICAÇÃO:
      //
      // Servidor para se comunicar com o encoder:
	ServerSocket server_enc (PORT_SIM2ENC);
	ServerSocket client_enc;
	cout << "Cria servidor server_enc... "; cout.flush();
	server_enc.accept (client_enc);
	cout << "conectou servidor server_enc    PASSO 1.2\n";

      // Servidor para se comunicar com o GPS:
	ServerSocket server_gps (PORT_SIM2GPS);
	ServerSocket client_gps;
	cout << "Cria servidor server_gps... "; cout.flush();
	server_gps.accept (client_gps);
	cout << "conectou servidor server_gps    PASSO 2.2\n";

      // Servidor para se comunicar com a bússola:
	ServerSocket server_bus (PORT_SIM2BUS);
	ServerSocket client_bus;
	cout << "Cria servidor server_bus... "; cout.flush();
	server_bus.accept (client_bus);
	cout << "conectou servidor server_bus    PASSO 3.2\n";
	std::cout << "\n--- Processo rodando...\n";
	//
      // --------------------------------------------

	q.zeros();
	
	for (int k=0; k<k_final; k++)
	{
		if (k>=k_final/2 && va>0) va = -va;
		u[0] = vl;
		u[1] = va;
//		u[1] = 0;

//		if(k>250) q[2] = M_PI/2;
//		if(k>500) q[2] = M_PI;
//		if(k>750) q[2] = 3*M_PI/2;

		q = f(q,u,T) + noise(sigma_x, sigma_y, sigma_theta);

		snprintf (buffer, sizeof(buffer), "%g %g %g %g %g", q[0], q[1], q[2], u[0], u[1]);

		client_enc >> msg;				// Aguarda requisição do encoder...
		client_enc << buffer;				// ... e envia o dado para ele.

		client_gps >> msg;				// Aguarda requisição do GPS...
		client_gps << buffer;				// ... e envia o dado para ele.

		client_bus >> msg;				// Aguarda requisição da bússola...
		client_bus << buffer;				// ... e envia o dado para ela.

		std::cout << " Step " << k << "\n";
	}
	std::cout << "\n--- Processo finalizado!\n\n" << '\x07';
	return 0;
}

/* simulador.cpp -- ESTA É A ÚLTIMA LINHA */
