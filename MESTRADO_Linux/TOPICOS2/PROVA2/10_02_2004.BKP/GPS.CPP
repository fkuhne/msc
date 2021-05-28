/* gps.cpp */

#include "defines.h"

#define m 2								// ordem da saída do GPS

#define sigma_gps_x 0.011
#define sigma_gps_y 0.011
#define var_gps_x (sigma_gps_x*sigma_gps_x)
#define var_gps_y (sigma_gps_y*sigma_gps_y)
	
CVECTOR<double> y(m);						// vetor de saída.
CVECTOR<double> v(m);						// ruído de medida.
CVECTOR<double> in(m);						// vetor de inovação.

MATRIX<double> K(n,m);						// ganho do EKF.
MATRIX<double> H(m,n);						// Jacobiano de h(x).
MATRIX<double> R(m,m);						// variâncias da medida.

int main (void)
{
	std::string msg;
	char carac [100];						// vetor auxiliar para separar os dados recebidos do servidor.
	char rcv_err[CHAR_SIZE];				// vetor para receber os erros 'e' e 'E' dos outros nodos.
	char snd_err[CHAR_SIZE];				// vetor para enviar os erros 'e' e 'E' dos outros nodos.
	char *next;							// ponteiro auxiliar.
	double T = 0.1;						// período de amostragem.

	// Cria arquivo de nome "xxx_gps", onde serão inseridos os dados:
	ofstream arq("./xxx_gps");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICAÇÃO:
	//
	
	// Cliente para comunicar com a simulação do processo "real":
	cout << "Aguardando conexão com o simulador... ";
	ClientSocket client_sim2gps ("localhost", PORT_SIM2GPS);
	cout << "cliente sim2gps conectado         PASSO 2.1\n";

	// Cliente para receber dados do servidor do encoder:
	cout << "Aguardando conexão com o encoder... ";
	ClientSocket client_enc2gps ("localhost", PORT_ENC2GPS);
	cout << "cliente enc2gps conectado           PASSO 4.1\n";

	client_enc2gps << "alive"; // envia sinalização para o encoder.

	// Servidor para comunicar com o cliente da bússola:
	ServerSocket server_gps2bus (PORT_GPS2BUS);
	ServerSocket server_bus2gps;
	cout << "Cria servidor gps2bus... ";
	server_gps2bus.accept (server_bus2gps);
	cout << "conectou servidor gps2bus                      PASSO 6.2\n";
	
	server_bus2gps >> msg; // aguarda sinalização da bússola.
	sleep(2);

	// Cliente para receber dados do servidor da bússola:
	cout << "Aguardando conexão com a bússola... ";
	ClientSocket client_bus2gps ("localhost", PORT_BUS2GPS);
	cout << "cliente bus2gps conectado           PASSO 8.1\n";

	client_bus2gps << "alive"; // envia sinalização para a bússola.

	client_enc2gps << "go"; // envia sinalização para o encoder.

	// Servidor para comunicar com o cliente do encoder:
	ServerSocket server_gps2enc (PORT_GPS2ENC);
	ServerSocket server_enc2gps;
	cout << "Cria servidor gps2enc... ";
	server_gps2enc.accept (server_enc2gps);
	cout << "conectou servidor gps2enc                      PASSO 9.2\n";

	server_enc2gps >> msg; // aguarda sinalização do encoder.

	//
	// --------------------------------------------------------

	std::cout << "\n--- Cliente conectado...\n\n";

	// Inicializa variáveis:
	x.zeros();
	x_est_global_k1.zeros();
	H.zeros(); H[0][0]=1; H[1][1]=1;
	I.eye();
	R[0][0] = var_gps_x; R[1][1] = var_gps_y;
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;

	client_sim2gps << "a";		// requisita dado ao servidor.

	while (true)
	{
		try
		{
			// Comunica com o servidor para receber os valores de [x y theta] do processo:
			client_sim2gps >> msg;			// recebe dado do servidor.

			memset(&carac[0], 0, sizeof(carac));
			for(int i=0; i<=msg.size(); i++){
				carac[i] = msg[i];
			}                                                   // estas linhas transformam
			x[0] = strtod(carac, &next);                        // o dado recebido (x,y,theta,u1,u2)
			x[1] = strtod(next, &next);                         // em 5 valores <double> que serão
			x[2] = strtod(next, &next);                         // alocados em um vetor [x;y;theta]
			u[0] = strtod(next, &next);                         // e em um vetor [u1 u2].
			u[1] = strtod(next, &next);

			v[0] = normal(0,sqrt(var_gps_x));
			v[1] = normal(0,sqrt(var_gps_y));
			y = H * x + v;                     // expressão de observação.

			// ------------------------------------------------------------------
			// Início do algoritmo do Filtro de Kalman Extendido Descentralizado:
			//
			K = P_global_k1 * H.t() * (H * P_global_k1 * H.t() + R).inv();
			in[0] = y[0] - x_est_global_k1[0];
			in[1] = y[1] - x_est_global_k1[1];
			x_est_local_k = x_est_global_k1 + K * in;
			P_local_k = (I - K * H) * P_global_k1;
			e_gps = P_local_k.inv() * x_est_local_k - P_global_k1.inv() * x_est_global_k1;
			E_gps = P_local_k.inv() - P_global_k1.inv();

			memset(&snd_err[0], 0, CHAR_SIZE); // zera o vetor de erro snd_err.

			// Grava os erros na variável snd_err:
			snprintf (snd_err, sizeof(snd_err), "%g %g %g %g %g %g %g %g %g %g %g %g", \
							E_gps[0][0],E_gps[0][1],E_gps[0][2], \
							E_gps[1][0],E_gps[1][1],E_gps[1][2], \
							E_gps[2][0],E_gps[2][1],E_gps[2][2], \
							e_gps[0],   e_gps[1],   e_gps[2]);

			client_bus2gps << snd_err;   // envia os erros para a bussola.
			client_enc2gps << snd_err;   // envia os erros para o encoder.

			msg = " ";
			server_enc2gps >> msg;  // recebe erros do encoder.

			memset(&rcv_err[0], 0, CHAR_SIZE); // zera o vetor de erro rcv_err.

			for(int i=0; i<=msg.size(); i++){
				rcv_err[i] = msg[i];
			}

			E_enc[0][0] = strtod(rcv_err, &next);  // grava erros do encoder.
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
			server_bus2gps >> msg; // recebe erros da bússola.

			memset(&rcv_err[0], 0, CHAR_SIZE); // zera o vetor de erro rcv_err.

			for(int i=0; i<=msg.size(); i++){
				rcv_err[i] = msg[i];
			}

			E_bus[0][0] = strtod(rcv_err, &next); // grava erros do encoder.
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


			client_sim2gps << snd_err; // send to controler the Es and es.

			// Bom, continuando com o EKF...

			P_global_k = (P_global_k1.inv() + E_enc + E_gps + E_bus).inv();
			x_est_global_k = P_global_k * (P_global_k1.inv() * x_est_global_k1 + e_enc + e_gps + e_bus);
			x_est_global_k1 = f(x_est_global_k,u,T);
			F = jacob_F(x_est_global_k,u,T);
			P_global_k1 = F * P_global_k * F.t() + Q;
			//
			// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
			// ---------------------------------------------------------------

		/*	client_bus2gps << "d";   //
			client_enc2gps << "d";   //
			server_enc2gps >> msg;      //
			server_bus2gps >> msg;      // */

                  // Escreve os dados no arquivo:
			arq
				<< x_est_global_k[0] << "\t" // 1
				<< x_est_global_k[1] << "\t" // 2
				<< x_est_global_k[2] << "\t" // 3

				<< P_global_k[0][0] << "\t"  // 4
				<< P_global_k[1][1] << "\t"  // 5
				<< P_global_k[2][2] << "\t"  // 6

				<< x[0] << "\t"               // 7
				<< x[1] << "\t"               // 8
				<< x[2] << "\t"               // 9

				<< "\n";
		}
		catch ( SocketException& e )
		{
			std::cout << "Exception was caught:" << e.description() << "\n";
			arq.close();

			return 0;
		}
	}

	arq.close();

	std::cout << "\n--- Cliente pronto!\n";

	return 0;
}
