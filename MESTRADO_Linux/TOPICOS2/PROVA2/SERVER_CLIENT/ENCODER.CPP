/* encoder.cpp */

#include "defines.h"

#define m 3								// ordem da saída do encoder

#define sigma_enc_x 1						// variância da medida x_enc.
#define sigma_enc_y 1						// variância da medida y_enc.
#define sigma_enc_theta 1.2					// variância da medida theta_enc.

CVECTOR<double> y(m);						// vetor de saída.
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
	//
	// --------------------------------------------------------

	// Inicializa variáveis:
	x.zeros();
	x_est_k1.zeros();
	H.eye();
	I.eye();
	R.zeros(); R[0][0] = sigma_enc_x*sigma_enc_x; R[1][1] = sigma_enc_y*sigma_enc_y; R[2][2] = sigma_enc_theta*sigma_enc_theta;
	Q.zeros(); Q[0][0] = sigma_x*sigma_x; Q[1][1] = sigma_y*sigma_y; Q[2][2] = sigma_theta*sigma_theta;
	P_k1.eye(); P_k1 = P_k1*1000.;

	while (true)
	{
		try
		{
			// Comunica com o servidor para receber os valores de [x y theta] do processo:
			client_sim2enc << "anything";   // requisita dado ao servidor.
			client_sim2enc >> msg;   // recebe dado do servidor.

			x = rcv_x(msg);
			u = rcv_u(msg);

			v[0] = normal(0,sigma_enc_x);
			v[1] = normal(0,sigma_enc_y);
			v[2] = normal(0,sigma_enc_theta);
			y = H * x + v;   // expressão de observação.

			// ------------------------------------------------------------------
			// Início do algoritmo do Filtro de Kalman Extendido Descentralizado:
			//
			// ESTIMAÇÃO:
			K = P_k1 * H.t() * (H * P_k1 * H.t() + R).inv();
			x_est_k = x_est_k1 + K * (y - x_est_k1);
			P_k = (I - K * H) * P_k1;
			// PREDIÇÃO:
			x_est_k1 = f(x_est_k,u,T);
			F = jacob_F(x_est_k,u,T);
			P_k1 = F * P_k * F.t() + Q;
			//
			// Fim do algoritmo do Filtro de Kalman Extendido Descentralizado.
			// ---------------------------------------------------------------

			// Escreve os dados no arquivo:
			arq
				<< x_est_k[0] << "\t"			// 1
				<< x_est_k[1] << "\t"			// 2
				<< x_est_k[2] << "\t"			// 3

				<< P_k[0][0] << "\t"			// 4
				<< P_k[1][1] << "\t"			// 5
				<< P_k[2][2] << "\t"			// 6

				<< x[0] << "\t"				// 7
				<< x[1] << "\t"				// 8
				<< x[2] << "\t"				// 9

				<< K[0][0] << "\t"			// 10
				<< K[1][1] << "\t"			// 11
				<< K[2][2] << "\t"			// 12

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
