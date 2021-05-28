/* bussola.cpp */

#include "defines.h"

#define m 1								// ordem da saída da bússola.

#define sigma_bus_theta 0.0185
#define var_bus_theta (sigma_bus_theta*sigma_bus_theta)

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

	// Cria arquivo de nome "xxx_bus", onde serão inseridos os dados:
	ofstream arq("./xxx_bus");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICAÇÃO:
	//

	// Cliente para comunicar com a simulação do processo "real":
	cout << "Aguardando conexão com o simulador... ";
	ClientSocket client_sim2bus ("localhost", PORT_SIM2BUS);
	cout << "cliente sim2bus conectado         PASSO 3.1\n";

	// Cliente para receber dados do servidor do encoder:
	cout << "Aguardando conexão com o encoder... ";
	ClientSocket client_enc2bus ("localhost", PORT_ENC2BUS);
	cout << "cliente enc2bus conectado           PASSO 5.1\n";

	client_enc2bus << "alive"; // envia sinalização para o encoder.

	// Cliente para receber dados do servidor do GPS:
	cout << "Aguardando conexão com o GPS... ";
	ClientSocket client_gps2bus ("localhost", PORT_GPS2BUS);
	cout << "cliente gps2bus conectado               PASSO 6.1\n";

	client_gps2bus << "alive"; // envia sinalização para o GPS.

	client_enc2bus << "go"; // envia sinalização para o encoder.

	// Servidor para comunicar com o cliente do encoder:
	ServerSocket server_bus2enc (PORT_BUS2ENC);
	ServerSocket server_enc2bus;
	cout << "Cria servidor bus2enc... ";
	server_bus2enc.accept (server_enc2bus);
	cout << "conectou servidor bus2enc                      PASSO 7.2\n";

	server_enc2bus >> msg; // aguarda sinalização do encoder;

	client_gps2bus << "alive"; // envia sinalização para o GPS.

	// Servidor para comunicar com o cliente do GPS:
	ServerSocket server_bus2gps (PORT_BUS2GPS);
	ServerSocket server_gps2bus;
	cout << "Cria servidor bus2gps... ";
	server_bus2gps.accept (server_gps2bus);
	cout << "conectou servidor bus2gps                      PASSO 8.2\n";

	server_gps2bus >> msg; // aguarda sinalização do GPS.

	//
	// --------------------------------------------------------

	std::cout << "\n--- Cliente conectado...\n\n";

	// Inicializa variáveis:
	x.zeros();
	x_est_global_k1.zeros();
	H.zeros(); H[0][2]=1;
	I.eye();
	R[0][0] = var_bus_theta;
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;

	client_sim2bus << "a"; // requisita dado ao servidor.

	while (true)
	{
		try
		{
			// Comunica com o servidor para receber os valores de [x y theta] do processo:
			client_sim2bus >> msg; // recebe dado do servidor.

			memset(&carac[0], 0, sizeof(carac));
			for(int i=0; i<=msg.size(); i++){
				carac[i] = msg[i];
			}                                   // estas linhas transformam
			x[0] = strtod(carac, &next);        // o dado recebido (x,y,theta,u1,u2)
			x[1] = strtod(next, &next);         // em 5 valores <double> que serão
			x[2] = strtod(next, &next);         // alocados em um vetor [x;y;theta]
			u[0] = strtod(next, &next);         // e em um vetor [u1 u2].
			u[1] = strtod(next, &next);

			v[0] = normal(0,sqrt(var_bus_theta));
			y = H * x + v;   // expressão de observação.

			// ------------------------------------------------------------------
			// Início do algoritmo do Filtro de Kalman Extendido Descentralizado:
			//
			K = P_global_k1 * H.t() * (H * P_global_k1 * H.t() + R).inv();
			in[0] = y[0] - x_est_global_k1[2];
			x_est_local_k = x_est_global_k1 + K * in;
			P_local_k = (I - K * H) * P_global_k1;
			e_bus = P_local_k.inv() * x_est_local_k - P_global_k1.inv() * x_est_global_k1;
			E_bus = P_local_k.inv() - P_global_k1.inv();

			memset(&snd_err[0], 0, CHAR_SIZE);  // zera o vetor de erro snd_err.

			// Grava os erros na variável snd_err:
			snprintf (snd_err, SEND_ERR_SIZE, "%g %g %g %g %g %g %g %g %g %g %g %g", \
				E_bus[0][0],E_bus[0][1],E_bus[0][2], \
				E_bus[1][0],E_bus[1][1],E_bus[1][2], \
				E_bus[2][0],E_bus[2][1],E_bus[2][2], \
				e_bus[0],   e_bus[1],   e_bus[2]);

			client_enc2bus << snd_err;   // envia os erros para o encoder.
			client_gps2bus << snd_err;   // envia os erros para o GPS.

			msg = " ";
			server_enc2bus >> msg;  // recebe erros do encoder.

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
			server_gps2bus >> msg;   // recebe erros do GPS.

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

			client_sim2bus << snd_err; // send to controler the Es and es.

			// Bom, continuando com o EKF...

			P_global_k = (P_global_k1.inv() + E_enc + E_gps + E_bus).inv();
			x_est_global_k = P_global_k * (P_global_k1.inv() * x_est_global_k1 + e_enc + e_gps + e_bus);
			x_est_global_k1 = f(x_est_global_k,u,T);
			F = jacob_F(x_est_global_k,u,T);
			P_global_k1 = F * P_global_k * F.t() + Q;
			//
			// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
			// ---------------------------------------------------------------

		/*	client_enc2bus << "d";   // encoder ready.
			client_gps2bus << "d";   // GPS ready.
			server_enc2bus >> msg;      // errors of encoder received.
			server_gps2bus >> msg;      // errors of GPS received.  */

			// Escreve dados no arquivo:
			arq
				<< x_est_global_k[0] << "\t"   // 1
				<< x_est_global_k[1] << "\t"   // 2
				<< x_est_global_k[2] << "\t"   // 3

				<< P_global_k[0][0] << "\t"   // 4
				<< P_global_k[1][1] << "\t"   // 5
				<< P_global_k[2][2] << "\t"   // 6

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
