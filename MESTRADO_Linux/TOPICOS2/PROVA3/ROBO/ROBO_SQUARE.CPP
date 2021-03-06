/* robo_square.cpp */
/* realiza uma trajet�ria quadrada */

#include "control_defines.h"

int main(void)
{
	// Inicializa��o das vari�veis:
	x.zeros(); x[0]=4; x[1]=4; x[2]=3*M_PI/2;					// configura��o inicial do rob�.
	x_ref.zeros();
	H.eye();
	r_r[0] = x_ref_p; r_r[1] = y_ref_p; r_r[2] = 0;
	K.eye(); K[0][0] = k_x; K[1][1] = k_y;

	double vl = 0.3;							// velocidade linear do rob�.
	double va = 0;							// velocidade angular em funcao da velocidade tangencial e o raio.
	double T = 0.1;							// periodo de amostragem de 100 ms.
	double k_final = 1000.;						// tempo total dividido pelo periodo de amostragem.
	
	// Cria arquivo de nome "traj", onde ser�o inseridos os dados
	ofstream arq("quadrado_sem_k");

	for(int k = 0; k<k_final; k++)
	{
		// Sa�da atual:
		y = H * x ;//+ noise(sigma_out_x, sigma_out_y, sigma_out_theta);

		// Sa�da no ponto de refer�ncia com rela��o ao sistema m�vel do rob�:
		y_r = go_to_ref_point(y, r_r);

		// Sa�da de refer�ncia:
		y_ref = go_to_ref_point(H * x_ref, r_r);

		// Lei de controle linear:
		v = control(y_ref, y_r, K);

		// Realimenta��o linearizante:
		u = linearization(y_r, r_r, v);

		// Trajet�ria real:
		x = f(x,u,T);// + noise(sigma_x, sigma_y, sigma_theta);

		// Trajet�ria de refer�ncia:
		u_ref[0] = vl; u_ref[1] = va;

		if(k>250) x_ref[2]=M_PI/2;
		if(k>500) x_ref[2]=M_PI;
		if(k>750) x_ref[2]=3*M_PI/2;
		
		x_ref = f(x_ref,u_ref,T);

		// Insere os dados no arquivo "traj", separados por tab:
		arq
			<< y_r[0] << "\t"					// coluna 1 - x real
			<< y_r[1] << "\t"					// coluna 2 - y real
			<< y_r[2] << "\t"					// coluna 3 - theta real

			<< y_ref[0] << "\t"				// coluna 4 - x refer�ncia
			<< y_ref[1] << "\t"				// coluna 5 - y refer�ncia

			<< u[0] << "\t"					// coluna 6 - controle
			<< u[1] << "\t"					// coluna 7 - controle

			<< y_ref[0] - y_r[0] << "\t"			// coluna 8 - erro em x
			<< y_ref[1] - y_r[1] << "\t"			// coluna 9 - erro em y
                  
			<< "\n";
	}

	// Fechar o arquivo onde foram gravados os estados e sa�das a cada instante de amostragem.
	arq.close();

      // Executa o GnuPlot conforme o script trajplot.gnu:
	execlp("gnuplot", "gnuplot", "plot_square.gnu", NULL);	
	
	return 0;
}

// ESTA � A �LTIMA LINHA DO PROGRAMA
