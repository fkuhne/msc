/* robo_eight.cpp */
/* realiza uma trajetória em forma de oito */

#include "control_defines.h"


int main(void)
{
	// Inicialização das variáveis:
	x.zeros(); x[0]=0; x[1]=2; x[2]=3*M_PI/2;				// configuração inicial do robô.
	x_ref.zeros();
	H.eye();
	r_r[0] = x_ref_p; r_r[1] = y_ref_p; r_r[2] = 0;
	K.eye(); K[0][0] = k_x; K[1][1] = k_y;

	double vl = 0.3;							// velocidade linear do robô.
	double r = 2;							// raio da trajetória do robô.
	double va = vl/r;							// velocidade angular em funcao da velocidade tangencial e o raio.
	double T = 0.1;							// periodo de amostragem de 100 ms.
	double k_final = (4*M_PI/va)/T;				// tempo total dividido pelo periodo de amostragem.

	// Cria arquivo de nome "traj", onde serão inseridos os dados
	ofstream arq("oito_sem_k");

	for(int k = 0; k<k_final; k++)
	{
		// Saída atual:
		y = H * x;// + noise(sigma_out_x, sigma_out_y, sigma_out_theta);

		// Saída no ponto de referência com relação ao sistema móvel do robô:
		y_r = go_to_ref_point(y, r_r);


		// Saída de referência:
		y_ref = go_to_ref_point(H * x_ref, r_r);
		
		// Lei de controle linear:
		v = control(y_ref, y_r, K);

		// Realimentação linearizante:
		u = linearization(y_r, r_r, v);

		// Trajetória real:
		x = f(x,u,T);// + noise(sigma_x, sigma_y, sigma_theta);

		// Trajetória de referência:
		if(k>=k_final/2 && va>0) va = -va;
		u_ref[0] = vl; u_ref[1] = va;
		
		x_ref = f(x_ref,u_ref,T);

		
		// Insere os dados no arquivo "traj", separados por tab:
		arq
			<< y_r[0] << "\t"					// coluna 1 - x real
			<< y_r[1] << "\t"					// coluna 2 - y real
			<< y_r[2] << "\t"					// coluna 3 - theta real

			<< y_ref[0] << "\t"				// coluna 4 - x referência
			<< y_ref[1] << "\t"				// coluna 5 - y referência

			<< u[0] << "\t"					// coluna 6 - controle
			<< u[1] << "\t"					// coluna 7 - controle

			<< y_ref[0] - y_r[0] << "\t"			// coluna 8 - erro em x
			<< y_ref[1] - y_r[1] << "\t"			// coluna 9 - erro em y

/*			<< d1[0] << "\t"					// 10
			<< d1[1] << "\t"					// 11
			<< d1[2] << "\t"					// 12
			
			<< d2[0] << "\t"					// 13
			<< d2[1] << "\t"					// 14
			<< d2[2] << "\t"					// 15
*/
			<< "\n";
	}

	// Fechar o arquivo onde foram gravados os estados e saídas a cada instante de amostragem.
	arq.close();

      // Executa o Gnuplot conforme o script trajplot.gnu:
	execlp("gnuplot", "gnuplot", "plot_eight.gnu", NULL);	
	
	return 0;
}

// ESTA É A ÚLTIMA LINHA DO PROGRAMA
