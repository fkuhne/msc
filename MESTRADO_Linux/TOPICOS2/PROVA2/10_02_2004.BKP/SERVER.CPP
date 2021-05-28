/* server.cpp */

#include "defines.h"

int main (void)
{
	std::string msg;						// mensagem recebida dos clientes.
//	char carac [100];						// vetor auxiliar para separar os dados recebidos do servidor.
	char rcv_err[CHAR_SIZE];				// vetor para receber os erros 'e' e 'E' dos outros nodos.
//	char snd_err[CHAR_SIZE];				// vetor para enviar os erros 'e' e 'E' dos outros nodos.
	char *next;							// ponteiro auxiliar.
	char buffer [400];					// char para enviar [x y theta].

	double vl = 0.3;						// velocidade linear do robô.
	double r = 2;						// raio da trajetória.
	double va = vl/r;						// velocidade angular do robô.
	double T = 0.1;						// período de amostragem.
	double k_final = (4*M_PI/va)/T;			// tempo final de simulação (8 completo).
	double k_meio = k_final/2;				// metade da trajetória (qdo inverte a va).

	// Cria arquivo de nome "xxx_ctrl", onde serão inseridos os dados:
	ofstream arq("./xxx_ctrl");

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
	//
      // --------------------------------------------
	
	std::cout << "\n--- Processo rodando...\n";

	x.zeros();

	x_est_global_k1.zeros();
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;


	client_enc >> msg;
	client_gps >> msg;
	client_bus >> msg;

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

		client_enc << buffer;				// ... e envia o dado para o encoder.
		client_gps << buffer;				// ... e envia o dado para o GPS.
		client_bus << buffer;				// ... e envia o dado para a bússola.

		msg = " ";
		client_enc >> msg;				// Aguarda requisição do encoder...
		memset(&rcv_err[0], 0, CHAR_SIZE);  // zera o vetor de erro rcv_err.

		for(int i=0; i<=msg.size(); i++){
			rcv_err[i] = msg[i];
		}

		E_enc[0][0] = strtod(rcv_err, &next);   // grava erros do encoder.
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



		msg = " ";
		client_gps >> msg;				// Aguarda requisição do GPS...
		memset(&rcv_err[0], 0, CHAR_SIZE); // zera o vetor de erro rcv_err.

		for(int i=0; i<=msg.size(); i++){
			rcv_err[i] = msg[i];
		}

		E_gps[0][0] = strtod(rcv_err, &next);   // grava erros do encoder.
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



		msg = " ";
		client_bus >> msg;				// Aguarda requisição da bússola...
		memset(&rcv_err[0], 0, CHAR_SIZE); // zera o vetor de erro rcv_err.

		for(int i=0; i<=msg.size(); i++){
			rcv_err[i] = msg[i];
		}

		E_bus[0][0] = strtod(rcv_err, &next);   // grava erros do encoder.
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


		P_global_k = (P_global_k1.inv() + E_enc + E_gps + E_bus).inv();
		x_est_global_k = P_global_k * (P_global_k1.inv() * x_est_global_k1 + e_enc + e_gps + e_bus);
		x_est_global_k1 = f(x_est_global_k,u,T);
		F = jacob_F(x_est_global_k,u,T);
		P_global_k1 = F * P_global_k * F.t() + Q;
		//
		// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
		// ---------------------------------------------------------------

		// Escreve os dados no arquivo:
		arq
			<< x_est_global_k[0] << "\t"        // 1
			<< x_est_global_k[1] << "\t"        // 2
			<< x_est_global_k[2] << "\t"        // 3

			<< P_global_k[0][0] << "\t"         // 4
			<< P_global_k[1][1] << "\t"         // 5
			<< P_global_k[2][2] << "\t"         // 6

			<< x[0] << "\t"                     // 7
			<< x[1] << "\t"                     // 8
			<< x[2] << "\t"                     // 9

			<< "\n";


		std::cout << " Passo " << k << "\n";
	}

	arq.close();
	std::cout << "\n--- Processo finalizado!\n\n";

	return 0;
}

/* server.cpp -- ESTA É A ÚLTIMA LINHA */
