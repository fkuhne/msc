#include "defines.h"

int main(void)
{
	double vl = 0.3;						// velocidade linear do robô.
	double r = 2;						// raio da trajetória do robô.
	double va = vl/r;						// velocidade angular em funcao da velocidade tangencial e o raio.
	double T = 0.1;						// periodo de amostragem de 100 ms.
	double k_final = (4*M_PI/va)/T;			// tempo total para completar os dois circulos dividido pelo periodo de amostragem.
	double k_meio = k_final/2;

	// Inicialização das matrizes e variáveis.

	I.eye();
	Q.zeros(); Q[0][0] = var_x; Q[1][1] = var_y; Q[2][2] = var_theta;

	x_enc_est_global_k1.zeros();
	H_enc.eye();
	R_enc.zeros(); R_enc[0][0] = var_enc_x; R_enc[1][1] = var_enc_y; R_enc[2][2] = var_enc_theta;
	P_enc_global_k1.eye(); P_enc_global_k1 = P_enc_global_k1*1000.;

	x_gps_est_global_k1.zeros();
	H_gps[0][0] = 1; H_gps[1][1] = 1;
	R_gps.zeros(); R_gps[0][0] = var_gps_x; R_gps[1][1] = var_gps_y;
	P_gps_global_k1.eye(); P_gps_global_k1 = P_gps_global_k1*1000.;

	x_bus_est_global_k1.zeros();
	H_bus[0][2] = 1;
	R_bus[0][0] = var_bus_theta;
	P_bus_global_k1.eye(); P_bus_global_k1 = P_bus_global_k1*1000.;


	// Cria arquivo de nome "xxx", onde serão inseridos os dados
	ofstream arq("xxx");

	for(int k = 0; k<k_final; k++)
	{

		// Se já fechou o primeiro circulo do 8, inverte a velocidade angular.
		if(k>=k_meio && va>0) va = -va;

		u[0] = vl;							// velocidade linear
		u[1] = va;							// velocidade angular

		// Ruído de processo
		w[0] = normal(0,sqrt(var_x));
		w[1] = normal(0,sqrt(var_y));
		w[2] = normal(0,sqrt(var_theta));

		// Trajetória do robô (em 8) sem o filtro de Kalman.
		x = f(x,u,T) + w;

		// Ruído de medida
		v_enc[0] = normal(0,sqrt(var_enc_x));
		v_enc[1] = normal(0,sqrt(var_enc_y));
		v_enc[2] = normal(0,sqrt(var_enc_theta));

		v_gps[0] = normal(0,sqrt(var_gps_x));
		v_gps[1] = normal(0,sqrt(var_gps_y));

		v_bus[0] = normal(0,sqrt(var_bus_theta));

		// Gera a saida
		y_enc = H_enc * x + v_enc;
		y_gps = H_gps * x + v_gps;
		y_bus = H_bus * x + v_bus;


		// Passo 1 -    K(k) = P(k|k-1) * Ht(k) *inv(H(k) * P(k|k-1) * Ht(k) + R(k))
		K_enc = P_enc_global_k1 * H_enc.t() * (H_enc * P_enc_global_k1 * H_enc.t() + R_enc).inv();
		K_gps = P_gps_global_k1 * H_gps.t() * (H_gps * P_gps_global_k1 * H_gps.t() + R_gps).inv();
		K_bus = P_bus_global_k1 * H_bus.t() * (H_bus * P_bus_global_k1 * H_bus.t() + R_bus).inv();

            // Calcula a inovação
		in_enc = y_enc - x_enc_est_global_k1;
		in_gps[0] = y_gps[0] - x_gps_est_global_k1[0]; in_gps[1] = y_gps[1] - x_gps_est_global_k1[1];
		in_bus[0] = y_bus[0] - x_bus_est_global_k1[2];
            
		// Passo 2 -    ~x(k|k) = ^x(k|k-1) + K(k) * (y(k) - h(^x(k|k-1), u(k)))
		x_enc_est_local_k = x_enc_est_global_k1 + K_enc * in_enc;
		x_gps_est_local_k = x_gps_est_global_k1 + K_gps * in_gps;
		x_bus_est_local_k = x_bus_est_global_k1 + K_bus * in_bus;

		// Passo 3 -    ~P(k|k) = (I - K(k) * H(k)) * P(k|k-1)
		P_enc_local_k = (I - K_enc * H_enc) * P_enc_global_k1;
		P_gps_local_k = (I - K_gps * H_gps) * P_gps_global_k1;
		P_bus_local_k = (I - K_bus * H_bus) * P_bus_global_k1;

		// Passo 4 -    e(k) = inv(~P(k|k)) * ~x(k|k) - inv(P(k|k-1)) * ^x(k|k-1)
		e_enc = P_enc_local_k.inv() * x_enc_est_local_k - P_enc_global_k1.inv() * x_enc_est_global_k1;
		e_gps = P_gps_local_k.inv() * x_gps_est_local_k - P_gps_global_k1.inv() * x_gps_est_global_k1;
		e_bus = P_bus_local_k.inv() * x_bus_est_local_k - P_bus_global_k1.inv() * x_bus_est_global_k1;

		// Passo 5 -    E(k) = inv(~P(k|k)) - inv(P(k|k-1))
		E_enc = P_enc_local_k.inv() - P_enc_global_k1.inv();
		E_gps = P_gps_local_k.inv() - P_gps_global_k1.inv();
		E_bus = P_bus_local_k.inv() - P_bus_global_k1.inv();

		// Passo 6 -    P(k|k) = inv(inv(P(k|k-1)) + E_enc + E_gps + E_bus)
		P_enc_global_k = (P_enc_global_k1.inv() + E_enc + E_gps + E_bus).inv();
		P_gps_global_k = (P_gps_global_k1.inv() + E_enc + E_gps + E_bus).inv();
		P_bus_global_k = (P_bus_global_k1.inv() + E_enc + E_gps + E_bus).inv();

		// Passo 7 -    ^x(k|k) = P(k|k) * ( inv(P(k|k-1)) * ^x(k|k-1) + e_enc + e_gps + e_bus )
		x_enc_est_global_k = P_enc_global_k * (P_enc_global_k1.inv() * x_enc_est_global_k1 + e_enc + e_gps + e_bus);
		x_gps_est_global_k = P_gps_global_k * (P_gps_global_k1.inv() * x_gps_est_global_k1 + e_enc + e_gps + e_bus);
		x_bus_est_global_k = P_bus_global_k * (P_bus_global_k1.inv() * x_bus_est_global_k1 + e_enc + e_gps + e_bus);

		// Passo 8 -    ^x(k+1|k) = f(^x(k|k), u(k))
		x_enc_est_global_k1 = f(x_enc_est_global_k,u,T);
		x_gps_est_global_k1 = f(x_gps_est_global_k,u,T);
		x_bus_est_global_k1 = f(x_bus_est_global_k,u,T);

            // Calcula o jacobiano de f(x,u)
		F_enc = jacob_F(x_enc_est_global_k,u,T);
		F_gps = jacob_F(x_gps_est_global_k,u,T);
		F_bus = jacob_F(x_bus_est_global_k,u,T);

		// Passo 9 -    P(k+1|k) = F(k) * P(k|k) * Ft(k) + Q(k)
		P_enc_global_k1 = F_enc * P_enc_global_k * F_enc.t() + Q;
		P_gps_global_k1 = F_gps * P_gps_global_k * F_gps.t() + Q;
		P_bus_global_k1 = F_bus * P_bus_global_k * F_bus.t() + Q;


		// Insere os dados no arquivo "xxx", separados por tab:
		arq
			<< x_enc_est_global_k[0] << "\t"		// 1
			<< x_enc_est_global_k[1] << "\t"		// 2
			<< x_enc_est_global_k[2] << "\t"		// 3

			<< x_gps_est_global_k[0] << "\t"		// 4
			<< x_gps_est_global_k[1] << "\t"		// 5
			<< x_gps_est_global_k[2] << "\t"		// 6

			<< x_bus_est_global_k[0] << "\t"		// 7
			<< x_bus_est_global_k[1] << "\t"		// 8
			<< x_bus_est_global_k[2] << "\t"		// 9

			<< P_enc_global_k[0][0] << "\t"		// 10
			<< P_enc_global_k[1][1] << "\t"		// 11
			<< P_enc_global_k[2][2] << "\t"		// 12

			<< P_gps_global_k[0][0] << "\t"		// 13
			<< P_gps_global_k[1][1] << "\t"		// 14
			<< P_gps_global_k[2][2] << "\t"		// 15

			<< P_bus_global_k[0][0] << "\t"		// 16
			<< P_bus_global_k[1][1] << "\t"		// 17
			<< P_bus_global_k[2][2] << "\t"		// 18

			<< x[0] << "\t"					// 19
			<< x[1] << "\t"					// 20
			<< x[2] << "\t"					// 21

			<< "\n";

	}

	// Fechar o arquivo onde foram gravados os estados e saídas a cada instante de amostragem.
	arq.close();

	return 0;
}

// ESTA É A ÚLTIMA LINHA DO PROGRAMA
