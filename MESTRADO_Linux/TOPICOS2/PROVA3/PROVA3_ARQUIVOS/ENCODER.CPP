/* encoder.cpp */

#include "defines.h"

#define m 3								// ordem da sa�da do encoder

#define sigma_enc_x 0.5						// desvio padr�o da medida x_enc.
#define sigma_enc_y 0.5						// desvio padr�o da medida y_enc.
#define sigma_enc_theta 0.6					// desvio padr�o da medida theta_enc.
#define var_enc_x (sigma_enc_x*sigma_enc_x)
#define var_enc_y (sigma_enc_y*sigma_enc_y)
#define var_enc_theta (sigma_enc_theta*sigma_enc_theta)

CVECTOR<double> z(m);						// vetor de sa�da.
CVECTOR<double> v(m);						// ru�do de medida.
CVECTOR<double> in(m);						// vetor de inova��o.
MATRIX<double> K(n,m);						// ganho do EKF.
MATRIX<double> H(m,n);						// Jacobiano de h(x).
MATRIX<double> R(m,m);						// vari�ncias da medida.

int main (void)
{
	std::string msg;
	char snd_err[CHAR_SIZE];				// vetor para enviar os erros 'e' e 'E' dos outros nodos.
	double T = 0.1;						// per�odo de amostragem.

	// Cria arquivo de nome "xxx_enc", onde ser�o inseridos os dados:
	ofstream arq("./xxx_enc");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICA��O:
	//
	// Cliente para comunicar com a simula��o do processo "real":
	cout << "Aguardando conex�o com o controlador... ";
	ClientSocket client_sim2enc ("localhost", PORT_SIM2ENC);
	cout << "cliente sim2enc conectado         PASSO 1.1\n";

	// Servidor para receber dados do cliente do GPS:
	ServerSocket server_enc2gps (PORT_ENC2GPS);
	ServerSocket server_gps2enc;
	cout << "Cria servidor enc2gps... ";
	server_enc2gps.accept (server_gps2enc);
	cout << "conectou servidor enc2gps                      PASSO 4.2\n";

	server_gps2enc >> msg; // aguarda sinaliza��o do GPS.

	// Servidor para receber dados do cliente da b�ssola:
	ServerSocket server_enc2bus (PORT_ENC2BUS);
	ServerSocket server_bus2enc;
	cout << "Cria servidor enc2bus... ";
	server_enc2bus.accept (server_bus2enc);
	cout << "conectou servidor enc2bus                      PASSO 5.2\n";

	server_bus2enc >> msg; // aguarda sinaliza��o da b�ssola.
	sleep(1);
	
	// Cliente para enviar dados para o servidor da b�ssola:
	cout << "Aguardando conex�o com a b�ssola... ";
	ClientSocket client_bus2enc ("localhost", PORT_BUS2ENC);
	cout << "cliente bus2enc conectado           PASSO 7.1\n";

	client_bus2enc << "alive"; // envia sinaliza��o para a b�ssola.

	server_gps2enc >> msg; // aguarda sinaliza��o do GPS.
	sleep(1);

	// Cliente para enviar dados para o servidor do GPS:
	cout << "Aguardando conex�o com o GPS... ";
	ClientSocket client_gps2enc ("localhost", PORT_GPS2ENC);
	cout << "cliente gps2enc conectado               PASSO 9.1\n";

	client_gps2enc << "alive"; // envia sinaliza��o para o GPS.
	//
	// --------------------------------------------------------

	std::cout << "\n--- Cliente conectado...\n\n";

	// Inicializa vari�veis:
	q.zeros();
	q_est_global_k1.zeros();
	q_est_global_k1[0] = 2;
	q_est_global_k1[1] = 3;
	q_est_global_k1[2] = M_PI/2;
	H.eye();
	I.eye();
	R.zeros(); R[0][0] = var_enc_x; R[1][1] = var_enc_y; R[2][2] = var_enc_theta;
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;

	client_sim2enc << "a";
	
	while (true)
	{
		try
		{
			// Comunica com o servidor para receber os valores de [x y theta] do processo:
			client_sim2enc >> msg;   // recebe dado do servidor.
			q = rcv_q(msg);
			u = rcv_u(msg);

			z = H * q + noise(sigma_enc_x, sigma_enc_y, sigma_enc_theta);   // express�o de observa��o.

			// ------------------------------------------------------------------
			// In�cio do algoritmo do Filtro de Kalman Extendido Descentralizado:
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
			client_sim2enc << snd_err;   // envia os dados para o simulador.


			server_gps2enc >> msg;   // recebe erros do GPS.
			E_gps = rcv_E(msg);
			e_gps = rcv_e(msg);

                  
			server_bus2enc >> msg;   // recebe erros da bussola.
			E_bus = rcv_E(msg);
			e_bus = rcv_e(msg);


			// Bom, continuando com o EKF...

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
			std::cout << "Exception was caught: " << e.description() << "\n";
			arq.close();
			std::cout << "\n--- Cliente pronto!\n";
			return 0;
		}
	}
	return 0;
}
