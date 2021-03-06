/* bussola.cpp */

#include "defines.h"

#define m 1								// ordem da sa�da da b�ssola.

#define sigma_bus_theta 0.0185				// vari�ncia da medida x_bus.

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

	// Cria arquivo de nome "xxx_bus", onde ser�o inseridos os dados:
	ofstream arq("xxx_bus");

	// -------------------------------------------------------
	// MECANISMOS DE COMUNICA��O:
	//
	// Cliente para comunicar com a simula��o do processo "real":
	cout << "Aguardando conex�o com o simulador... ";
	ClientSocket client_sim2bus ("localhost", PORT_SIM2BUS);
	cout << "cliente sim2bus conectado         PASSO 3.1\n";

	// Cliente para receber dados do servidor do encoder:
	cout << "Aguardando conex�o com o encoder... ";
	ClientSocket client_enc2bus ("localhost", PORT_ENC2BUS);
	cout << "cliente enc2bus conectado           PASSO 5.1\n";
	client_enc2bus << "alive"; // envia sinaliza��o para o encoder.

	// Cliente para receber dados do servidor do GPS:
	cout << "Aguardando conex�o com o GPS... ";
	ClientSocket client_gps2bus ("localhost", PORT_GPS2BUS);
	cout << "cliente gps2bus conectado               PASSO 6.1\n";
	client_gps2bus << "alive"; // envia sinaliza��o para o GPS.
	client_enc2bus << "go"; // envia sinaliza��o para o encoder.

	// Servidor para comunicar com o cliente do encoder:
	ServerSocket server_bus2enc (PORT_BUS2ENC);
	ServerSocket server_enc2bus;
	cout << "Cria servidor bus2enc... ";
	server_bus2enc.accept (server_enc2bus);
	cout << "conectou servidor bus2enc                      PASSO 7.2\n";
	server_enc2bus >> msg; // aguarda sinaliza��o do encoder;
	client_gps2bus << "alive"; // envia sinaliza��o para o GPS.

	// Servidor para comunicar com o cliente do GPS:
	ServerSocket server_bus2gps (PORT_BUS2GPS);
	ServerSocket server_gps2bus;
	cout << "Cria servidor bus2gps... ";
	server_bus2gps.accept (server_gps2bus);
	cout << "conectou servidor bus2gps                      PASSO 8.2\n";
	server_gps2bus >> msg; // aguarda sinaliza��o do GPS.
	std::cout << "\n--- B�ssola conectada...\n\n";
	//
	// --------------------------------------------------------

	// Inicializa vari�veis:
	q.zeros();
	q_est_global_k1.zeros();
	H.zeros(); H[0][2]=1;
	I.eye();
	R[0][0] = sigma_bus_theta*sigma_bus_theta;
	Q.zeros(); Q[0][0] = sigma_x*sigma_x; Q[1][1] = sigma_y*sigma_y; Q[2][2] = sigma_theta*sigma_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;

	while (true)
	{
		try
		{
			// Comunica com o servidor para receber os valores de [x y theta] do processo:
			client_sim2bus << "anything"; // requisita dado ao servidor.
			client_sim2bus >> msg; // recebe dado do servidor.

			q = rcv_q(msg);
			u = rcv_u(msg);

			v[0] = normal(0,sigma_bus_theta);
			z = H * q + v;   // express�o de observa��o.

			// ------------------------------------------------------------------
			// In�cio do algoritmo do Filtro de Kalman Extendido Descentralizado:
			//
			K = P_global_k1 * H.t() * (H * P_global_k1 * H.t() + R).inv();
			in[0] = z[0] - q_est_global_k1[2];
			q_est_local_k = q_est_global_k1 + K * in;
			P_local_k = (I - K * H) * P_global_k1;
			e_bus = P_local_k.inv() * q_est_local_k - P_global_k1.inv() * q_est_global_k1;
			E_bus = P_local_k.inv() - P_global_k1.inv();

			memset(&snd_err[0], 0, CHAR_SIZE);  // zera o vetor de erro snd_err.
			snprintf (snd_err, SEND_ERR_SIZE, "%g %g %g %g %g %g %g %g %g %g %g %g", \
				E_bus[0][0],E_bus[0][1],E_bus[0][2], \
				E_bus[1][0],E_bus[1][1],E_bus[1][2], \
				E_bus[2][0],E_bus[2][1],E_bus[2][2], \
				e_bus[0],   e_bus[1],   e_bus[2]);

			client_enc2bus << snd_err;   // envia os erros para o encoder.
			client_gps2bus << snd_err;   // envia os erros para o GPS.

			server_enc2bus >> msg;  // recebe erros do encoder.
			E_enc = rcv_E(msg);
			e_enc = rcv_e(msg);

			server_gps2bus >> msg;   // recebe erros do GPS.
			E_gps = rcv_E(msg);
			e_gps = rcv_e(msg);

			// Bom, continuando com o EKF...

			P_global_k = (P_global_k1.inv() + E_enc + E_gps + E_bus).inv();
			q_est_global_k = P_global_k * (P_global_k1.inv() * q_est_global_k1 + e_enc + e_gps + e_bus);
			q_est_global_k1 = f(q_est_global_k,u,T);
			F = jacob_F(q_est_global_k,u,T);
			P_global_k1 = F * P_global_k * F.t() + Q;
			//
			// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
			// ---------------------------------------------------------------

			// Escreve dados no arquivo:
			arq
				<< q_est_global_k[0] << "\t"   // 1
				<< q_est_global_k[1] << "\t"   // 2
				<< q_est_global_k[2] << "\t"   // 3

				<< P_global_k[0][0] << "\t"   // 4
				<< P_global_k[1][1] << "\t"   // 5
				<< P_global_k[2][2] << "\t"   // 6

				<< q[0] << "\t"               // 7
				<< q[1] << "\t"               // 8
				<< q[2] << "\t"               // 9

				<< "\n";
		}
		catch ( SocketException& e )
		{
			std::cout << "Exception was caught:" << e.description() << "\n";
			arq.close();
			std::cout << "\n--- B�ssola finalizada!\n";
			return 0;
		}
	}
	arq.close();
	return 0;
}

/* bussola.cpp --- ESTA � A �LTIMA LINHA */
