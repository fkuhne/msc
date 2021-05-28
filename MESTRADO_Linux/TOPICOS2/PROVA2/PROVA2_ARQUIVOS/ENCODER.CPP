/* encoder.cpp */

#include "defines.h"

#define m 3								// ordem da saída do encoder.

#define sigma_enc_x 0.5						// desvio padrão da medida x_enc.
#define sigma_enc_y 0.5						// desvio padrão da medida y_enc.
#define sigma_enc_theta 0.6					// desvio padrão da medida theta_enc.

CVECTOR<double> z(m);						// vetor de saída.
CVECTOR<double> v(m);						// ruído de medida.
CVECTOR<double> in(m);						// vetor de inovação.

MATRIX<double> K(n,m);						// ganho do EKF.
MATRIX<double> H(m,n);						// Jacobiano de h(x).
MATRIX<double> R(m,m);						// variâncias da medida.

int main (void)
{
	std::string msg;
	char snd_err[CHAR_SIZE];				// vetor para enviar os erros 'e' e 'E' dos outros nodos.
	double T = 0.1;						// período de amostragem.

	// Cria arquivo de nome "xxx_enc", onde serão inseridos os dados:
	ofstream arq("xxx_enc");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICAÇÃO:
	//
	// Cliente para comunicar com a simulação do processo "real":
	cout << "Aguardando conexão com o simulador... ";
	ClientSocket client_sim2enc ("localhost", PORT_SIM2ENC);
	cout << "cliente sim2enc conectado         PASSO 1.1\n";

	// Servidor para receber dados do cliente do GPS:
	ServerSocket server_enc2gps (PORT_ENC2GPS);
	ServerSocket server_gps2enc;
	cout << "Cria servidor enc2gps... ";
	server_enc2gps.accept (server_gps2enc);
	cout << "conectou servidor enc2gps                      PASSO 4.2\n";
	server_gps2enc >> msg; // aguarda sinalização do GPS.

	// Servidor para receber dados do cliente da bússola:
	ServerSocket server_enc2bus (PORT_ENC2BUS);
	ServerSocket server_bus2enc;
	cout << "Cria servidor enc2bus... ";
	server_enc2bus.accept (server_bus2enc);
	cout << "conectou servidor enc2bus                      PASSO 5.2\n";
	server_bus2enc >> msg; // aguarda sinalização da bússola.
	sleep(1);

	// Cliente para enviar dados para o servidor da bússola:
	cout << "Aguardando conexão com a bússola... ";
	ClientSocket client_bus2enc ("localhost", PORT_BUS2ENC);
	cout << "cliente bus2enc conectado           PASSO 7.1\n";
	client_bus2enc << "alive"; // envia sinalização para a bússola.
	server_gps2enc >> msg; // aguarda sinalização do GPS.
	sleep(1);

	// Cliente para enviar dados para o servidor do GPS:
	cout << "Aguardando conexão com o GPS... ";
	ClientSocket client_gps2enc ("localhost", PORT_GPS2ENC);
	cout << "cliente gps2enc conectado               PASSO 9.1\n";
	client_gps2enc << "alive"; // envia sinalização para o GPS.
	std::cout << "\n--- Encoder conectado...\n\n";
	//
	// --------------------------------------------------------

	// Inicializa variáveis:
	q.zeros();
	q_est_global_k1.zeros();
	H.eye();
	I.eye();
	R.zeros(); R[0][0] = sigma_enc_x*sigma_enc_x; R[1][1] = sigma_enc_y*sigma_enc_y; R[2][2] = sigma_enc_theta*sigma_enc_theta;
	Q.zeros(); Q[0][0] = sigma_x*sigma_x; Q[1][1] = sigma_y*sigma_y; Q[2][2] = sigma_theta*sigma_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;

	while (true)
	{
		try
		{
			// Comunica com o servidor para receber os valores de [x y theta] do processo:
			client_sim2enc << "anything";   // requisita dado ao servidor.
			client_sim2enc >> msg;   // recebe dado do servidor.

			q = rcv_q(msg);
			u = rcv_u(msg);

			z = H * q + noise(sigma_enc_x, sigma_enc_y, sigma_enc_theta);   // expressão de observação.

			// ------------------------------------------------------------------
			// Início do algoritmo do Filtro de Kalman Extendido Descentralizado:
			//
			K = P_global_k1 * H.t() * (H * P_global_k1 * H.t() + R).inv();
			in = z - q_est_global_k1;
			q_est_local_k = q_est_global_k1 + K * in;
			P_local_k = (I - K * H) * P_global_k1;
			e_enc = P_local_k.inv() * q_est_local_k - P_global_k1.inv() * q_est_global_k1;
			E_enc = P_local_k.inv() - P_global_k1.inv();

			memset(&snd_err[0], 0, CHAR_SIZE);   // zera o vetor de erro snd_err.
			snprintf (snd_err, sizeof(snd_err), "%g %g %g %g %g %g %g %g %g %g %g %g", \
				E_enc[0][0],E_enc[0][1],E_enc[0][2], \
				E_enc[1][0],E_enc[1][1],E_enc[1][2], \
				E_enc[2][0],E_enc[2][1],E_enc[2][2], \
				e_enc[0],   e_enc[1],   e_enc[2]);

			client_bus2enc << snd_err;   // envia os erros para a bussula.
			client_gps2enc << snd_err;   // envia os erros para o GPS.

			server_gps2enc >> msg;   // recebe erros do GPS.
			E_gps = rcv_E(msg);
			e_gps = rcv_e(msg);

			server_bus2enc >> msg;   // recebe erros da bussola.
			E_bus = rcv_E(msg);
			e_bus = rcv_e(msg);


			// Bom, continuando com o EKF...

			/* OBSERVAÇÃO:
			   Para observar a estimativa apenas do encoder, comente a soma dos erros
			    E_gps, E_bus, e_gps e e_bus nas duas expressões abaixo
			*/
			P_global_k = (P_global_k1.inv() + E_enc + E_gps + E_bus).inv();
			q_est_global_k = P_global_k * (P_global_k1.inv() * q_est_global_k1 + e_enc + e_gps + e_bus);
			q_est_global_k1 = f(q_est_global_k,u,T);
			F = jacob_F(q_est_global_k,u,T);
			P_global_k1 = F * P_global_k * F.t() + Q;
			//
			// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
			// ---------------------------------------------------------------

			// Escreve os dados no arquivo:
			arq
				<< q_est_global_k[0] << "\t"        // 1
				<< q_est_global_k[1] << "\t"        // 2
				<< q_est_global_k[2] << "\t"        // 3

				<< P_global_k[0][0] << "\t"         // 4
				<< P_global_k[1][1] << "\t"         // 5
				<< P_global_k[2][2] << "\t"         // 6

				<< q[0] << "\t"                     // 7
				<< q[1] << "\t"                     // 8
				<< q[2] << "\t"                     // 9

				<< "\n";
		}
		catch ( SocketException& e )
		{
			std::cout << "Exception was caught:" << e.description() << "\n";
			arq.close();
			std::cout << "\n--- Encoder finalizado!\n";
			return 0;
		}
	}
	arq.close();
	return 0;
}

/* encoder.cpp --- ESTA É A ÚLTIMA LINHA */
