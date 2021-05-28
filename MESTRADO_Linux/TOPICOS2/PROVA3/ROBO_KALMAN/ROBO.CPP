/* robo.cpp */
/* utiliza o EKF sem comunicação e controle linearizante */

#include "func.h"
#include "controls.h"
#include "normal.h"
#include "mmath.h"
#include <iostream.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>


//
// DEFINIÇÕES DO MODELO:
//
#define n 3								// ordem do sistema.
#define p 2								// ordem da entrada.
#define m 3								// ordem da saída

#define sigma_x 0.0015						// variância do estado x.
#define sigma_y 0.0015						// variância do estado y.
#define sigma_theta 0.00075					// variância do estado theta.

#define sigma_out_x 0.011					// desvio padrão da medida x.
#define sigma_out_y 0.011					// desvio padrão da medida y.
#define sigma_out_theta 0.018					// desvio padrão da medida theta.

CVECTOR<double> q(n);						// vetor de estados reais (recebido do servidor).
CVECTOR<double> u(p);						// vetor de entradas de controle.
CVECTOR<double> z(m);						// vetor de saída.
MATRIX<double> F(n,n);						// jacobiano de f(x,u).
MATRIX<double> H(m,n);						// jacobiano de h(x,u).

//
// DEFINIÇÕES DO CONTROLE:
//
#define m_c 2

#define x_ref_p 0.2						// ponto de referência em x com relação ao sistema móvel do robô.
#define y_ref_p 0							// ponto de referência em y com relação ao sistema móvel do robô.

#define l_x 5							// ganho da lei de controle linear.
#define l_y 5							// ganho da lei de controle linear.

CVECTOR<double> q_ref(n);					// trajetória de referência.
CVECTOR<double> u_ref(p);					// vetor de entradas de controle de referência.
CVECTOR<double> z_ref(m);					// saída de referência.
CVECTOR<double> r_r(m);						// ponto de referência.
CVECTOR<double> z_r(m);						// saída com relação ao ponto de referência.
CVECTOR<double> v(m_c);						// entrada de controle linear.
MATRIX<double> L(m_c,m_c);					// matriz de ganhos do controle linear.

//
// DEFINIÇÕES DO FILTRO DE KALMAN:
//
CVECTOR<double> q_est_k(n);					// estado local estimado em (k|k).
CVECTOR<double> q_est_k1(n);					// estado global estimado em (k|k-1).
MATRIX<double> K(n,m);						// ganho do EKF.
MATRIX<double> I(n,n);						// matriz identidade.
MATRIX<double> Q(n,n);						// matriz de covariâncias do processo.
MATRIX<double> R(n,n);
MATRIX<double> P_k(n,n);					// covariância global em (k|k).
MATRIX<double> P_k1(n,n);		      		// covariância global em (k|k-1).


//
// PROGRAMA PRINCIPAL:
//
int main(int argc, char *argv[])
{
	// Inicialização das variáveis:
	r_r[0] = x_ref_p; r_r[1] = y_ref_p; r_r[2] = 0;

	q.zeros(); q[1] =2 ;							// só é necessário caso não se use o EKF.
	q_est_k1.zeros(); q_est_k1[2] = 2;		 	// configuração inicial do robô.
	q_ref.zeros();
	I.eye();
	H.eye();
	L.eye(); L[0][0] = l_x; L[1][1] = l_y;
	R.zeros(); R[0][0] = sigma_out_x*sigma_out_x; R[1][1] = sigma_out_y*sigma_out_y; R[2][2] = sigma_out_theta*sigma_out_theta;
	Q.zeros(); Q[0][0] = sigma_x*sigma_x; Q[1][1] = sigma_y*sigma_y; Q[2][2] = sigma_theta*sigma_theta;
	P_k1.eye(); P_k1 = P_k1*1000.;

	double vl = 0.3;							// velocidade linear do robô.
	double r = 2;							// raio da trajetória do robô.
	double va = vl/r;							// velocidade angular em funcao da velocidade tangencial e o raio.
	double T = 0.1;							// periodo de amostragem de 100 ms.
	double k_final = (4*M_PI/va)/T;				// tempo total dividido pelo periodo de amostragem.

/*	if (argv[1] == "square")
	{
		va = 0;
		k_final = 1000.;
	}
*/

	// Cria arquivo de nome "traj", onde serão inseridos os dados
	ofstream arq("traj");

	for(int k = 0; k<k_final; k++)
	{
		// Saída atual:
		z = H * q + noise(sigma_out_x, sigma_out_y, sigma_out_theta);

		// Saída no ponto de referência com relação ao sistema móvel do robô:
		z_r = go_to_ref_point(z, r_r);

		// Saída de referência:
		z_ref = go_to_ref_point(H * q_ref, r_r);

		// Lei de controle linear:
		v = control(z_ref, z_r, L);

		// Realimentação linearizante:
		u = linearization(z_r, r_r, v);

		// Trajetória real:
		q = f(q,u,T) + noise(sigma_x, sigma_y, sigma_theta);

		// Trajetória de referência:
//		if (argv[1] == "eight")
//		{
			if(k>=k_final/2 && va>0) va = -va;
//		}
//		else if (argv[1] == "square")
//		{
//			if (k>250) q_ref[2] = M_PI/2;
//			if (k>500) q_ref[2] = M_PI;
//			if (k>750) q_ref[2] = 3*M_PI/2;
//		}
		u_ref[0] = vl; u_ref[1] = va;
		q_ref = f(q_ref,u_ref,T);

            // EKF:
		K = P_k1 * H.t() * (H * P_k1 * H.t() + R).inv();
		q_est_k = q_est_k1 + K * (z - q_est_k1);
		P_k = (I - K * H) * P_k1;

		F = jacob_F(q_est_k,u,T);
		P_k1 = F * P_k * F.t() + Q;
		q_est_k1 = f(q_est_k,u,T);
		
		z = go_to_center_point(z_r, r_r);
		CVECTOR<double> zz_ref = go_to_center_point(z_ref, r_r);

		// Insere os dados no arquivo "traj", separados por tab:
		arq
			<< z_r[0] << "\t"					// coluna 1 - x real
			<< z_r[1] << "\t"					// coluna 2 - y real
			<< z_r[2] << "\t"					// coluna 3 - theta real

			<< z_ref[0] << "\t"				// coluna 4 - x referência
			<< z_ref[1] << "\t"				// coluna 5 - y referência

			<< u[0] << "\t"					// coluna 6 - controle
			<< u[1] << "\t"					// coluna 7 - controle

			<< z_ref[0] - z_r[0] << "\t"			// coluna 8 - erro em theta
			<< z_ref[1] - z_r[1] << "\t"			// coluna 9 - erro em theta

			<< z[0] << "\t"					// coluna 10 - x ref
			<< z[1] << "\t"					// coluna 11 - y ref
			<< z[2] << "\t"					// coluna 12 - theta ref

			<< zz_ref[0] << "\t"				// coluna 13 - x referência
			<< zz_ref[1] << "\t"				// coluna 14 - y referência

			<< "\n";
	}

	// Fechar o arquivo onde foram gravados os estados e saídas a cada instante de amostragem.
	arq.close();

      // Executa o GnuPlot conforme o script trajplot.gnu:
	execlp("gnuplot", "gnuplot", "trajplot.gnu", NULL);	
	
	return 0;
}

// ESTA É A ÚLTIMA LINHA DO PROGRAMA
