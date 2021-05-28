/* controlador.cpp */

#include "control_defines.h"
#include "normal.h"
#include "controls.h"
#include "func.h"

int main (void)
{
	std::string msg;						// mensagem recebida dos clientes.
	char buffer [400];					// char para enviar [x y theta].

	double vl = 0.3;						// velocidade linear do robô.
	double r = 2;						// raio da trajetória.
//	double va = vl/r;						// velocidade angular do robô.
	double T = 0.1;						// período de amostragem.
//	double k_final = (4*M_PI/va)/T;			// tempo final de simulação (8 completo).
	double k_final = 1000;

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

	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;
	P_global_k1.eye(); P_global_k1 = P_global_k1*1000.;
	H.eye();
	q.zeros();
	q_ref.zeros();
	q_est_global_k1.zeros();

	// Configuração inicial do robô:
	q_est_global_k1[0] = 2;
	q_est_global_k1[1] = 3;
	q_est_global_k1[2] = M_PI/2;
	q_est = q_est_global_k1;
	q = q_est;
	
	// Ponto de referência R:
	r_r[0] = x_p_ref; r_r[1] = y_p_ref; r_r[2] = 0;

	// Ganho linear:
	L.zeros(); L[0][0] = l_x; L[1][1] = l_y;

	client_enc >> msg;
	client_gps >> msg;
	client_bus >> msg;

	for (int k=0; k<k_final; k++)
	{
		// Estado estimado do ponto C:
		q_est = q_est_global_k1;

		// Estado estimado do ponto R:
		q_r_est = go_to_ref_point(q_est, r_r);

		// Estado de referência do ponto R:
		q_r_ref = go_to_ref_point(q_ref, r_r);

		// Lei de controle linear:
		v = control(q_r_ref, q_r_est, L);

		// Realimentação linearizante:
		u = linearization(q_r_est, r_r, v);

		// Calculo do estado real:
		q = f(q, u, T) + noise(sigma_x, sigma_y, sigma_theta);
		snprintf (buffer, sizeof(buffer), "%g %g %g %g %g", q[0], q[1], q[2], u[0], u[1]);

		// Controle de referência:
//		if (k>=k_final/2 && va>0) va = -va;
//		u_ref[0] = vl; u_ref[1] = va;
		u_ref[0] = vl; u_ref[1] = 0;
		if(k>250) q_ref[2] = M_PI/2;
		if(k>500) q_ref[2] = M_PI;
		if(k>750) q_ref[2] = 3*M_PI/2;	

            // Estado de referência do ponto C:
		q_ref = f(q_ref, u_ref, T);

		// Envia q(k) e u(k) para os sensores:
		client_enc << buffer;
		client_gps << buffer;
		client_bus << buffer;

		// Recebe erros do encoder:
		client_enc >> msg;
		E_enc = rcv_E(msg);
		e_enc = rcv_e(msg);

		// Recebe erros do GPS:
		client_gps >> msg;
		E_gps = rcv_E(msg);
		e_gps = rcv_e(msg);

		// Recebe erros da bússola:
		client_bus >> msg;
		E_bus = rcv_E(msg);
		e_bus = rcv_e(msg);

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
//			<< q_r_ref[0] << "\t"			// 1
//			<< q_r_ref[1] << "\t"			// 2
//			<< q_r_ref[2] << "\t"			// 3

//			<< q_r_est[0] << "\t"			// 4
//			<< q_r_est[1] << "\t"			// 5
//			<< q_r_est[2] << "\t"			// 6

//			<< u[0] << "\t"				// 7
//			<< u[1] << "\t"				// 8

//			<< P_global_k[0][0] << "\t"		// 9
//			<< P_global_k[1][1] << "\t"		// 10
//			<< P_global_k[2][2] << "\t"		// 11

			<< q_ref[0] << "\t"			// 12
			<< q_ref[1] << "\t"			// 13
			<< q_ref[2] << "\t"			// 14

			<< q_est[0] << "\t"			// 15
			<< q_est[1] << "\t"			// 16
			<< q_est[2] << "\t"			// 17
			
			<< "\n";

		std::cout << " Passo " << k << "\n";
	}
	std::cout << "\n--- Processo finalizado!\n\n";
//	execlp("gnuplot", "gnuplot", "trajplot.gnu", NULL);
	arq.close();
	return 0;
}

/* controlador.cpp -- ESTA É A ÚLTIMA LINHA */
