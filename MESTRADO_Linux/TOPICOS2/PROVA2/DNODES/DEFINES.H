#include "mmath.h"
#include "normal.h"
#include "func.h"
#include <stdio.h>
#include <fstream.h>

// Defini��es
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#define n 3									// ordem do sistema
#define p 2									// ordem da entrada
#define m_enc 3								// ordem da sa�da do encoder
#define m_gps 2								// ordem da sa�da do GPS
#define m_bus 1								// ordem da sa�da da b�ssola
	
#define var_x 0.0001							// vari�ncia do estado x
#define var_y 0.0001							// vari�ncia do estado y
#define var_theta 0.00008						// vari�ncia do estado theta

#define var_enc_x 0.003							// vari�ncia da medida x_enc
#define var_enc_y 0.003							// vari�ncia da medida y_enc
#define var_enc_theta 0.004						// vari�ncia da medida theta_enc
	
#define var_gps_x 0.001							// vari�ncia da medida x_gps
#define var_gps_y 0.001							// vari�ncia da medida y_gps

#define var_bus_theta 0.002						// vari�ncia da medida x_bus

/********* DECLARA��O DAS VARI�VEIS *******/

/* VARI�VEIS V�LIDAS PARA TODOS OS NODOS */
CVECTOR<double> x(n);							// vetor de estados reais
CVECTOR<double> u(p);							// vetor de entradas de controle
CVECTOR<double> w(n);							// ru�do de processo
MATRIX<double> I(n,n);							// matriz identidade
MATRIX<double> Q(n,n);							// vari�ncias do processo

/********* NODO 1 - ENCODERS *********/
CVECTOR<double> x_enc_est_local_k(n);				// estado local estimado em (k|k)
CVECTOR<double> x_enc_est_global_k(n);				// estado global estimado em (k|k)
CVECTOR<double> x_enc_est_global_k1(n);				// estado global estimado em (k|k-1)
CVECTOR<double> e_enc(n);						// erro de estima��o
CVECTOR<double> y_enc(m_enc);						// vetor de sa�da
CVECTOR<double> v_enc(m_enc);						// ru�do de medida
CVECTOR<double> in_enc(m_enc);					// vetor de inova��o
MATRIX<double> F_enc(n,n);						// jacobiano de f(x,u)
MATRIX<double> P_enc_local_k(n,n);					// covari�ncia local em (k|k)
MATRIX<double> P_enc_global_k(n,n);					// covari�ncia global em (k|k)
MATRIX<double> P_enc_global_k1(n,n);				// covari�ncia global em (k|k-1)
MATRIX<double> E_enc(n,n);						// covari�ncia do erro
MATRIX<double> K_enc(n,m_enc);					// ganho do EKF
MATRIX<double> H_enc(m_enc,n);					// Jacobiano de h(x)
MATRIX<double> R_enc(m_enc,m_enc);					// vari�ncias da medida

/********* NODO 2 - GPS *********/
CVECTOR<double> x_gps_est_local_k(n);				// estado local estimado em (k|k)
CVECTOR<double> x_gps_est_global_k(n);				// estado global estimado em (k|k)
CVECTOR<double> x_gps_est_global_k1(n);				// estado global estimado em (k|k-1)
CVECTOR<double> e_gps(n);						// erro de estima��o
CVECTOR<double> y_gps(m_gps);						// vetor de sa�da
CVECTOR<double> v_gps(m_gps);						// ru�do de medida
CVECTOR<double> in_gps(m_gps);					// vetor de inova��o
MATRIX<double> F_gps(n,n);						// jacobiano de f(x,u)
MATRIX<double> P_gps_local_k(n,n);					// covari�ncia local em (k|k)
MATRIX<double> P_gps_global_k(n,n);					// covari�ncia global em (k|k)
MATRIX<double> P_gps_global_k1(n,n);				// covari�ncia global em (k|k-1)
MATRIX<double> E_gps(n,n);						// covari�ncia do erro
MATRIX<double> K_gps(n,m_gps);					// ganho do EKF
MATRIX<double> H_gps(m_gps,n);					// Jacobiano de h(x)
MATRIX<double> R_gps(m_gps,m_gps);					// vari�ncias da medida

/********* NODO 3 - B�SSOLA *********/
CVECTOR<double> x_bus_est_local_k(n);				// estado local estimado em (k|k)
CVECTOR<double> x_bus_est_global_k(n);				// estado global estimado em (k|k)
CVECTOR<double> x_bus_est_global_k1(n);				// estado global estimado em (k|k-1)
CVECTOR<double> e_bus(n);						// erro de estima��o
CVECTOR<double> y_bus(m_bus);						// vetor de sa�da
CVECTOR<double> v_bus(m_bus);						// ru�do de medida
CVECTOR<double> in_bus(m_bus);					// vetor de inova��o
MATRIX<double> F_bus(n,n);						// jacobiano de f(x,u)
MATRIX<double> P_bus_local_k(n,n);					// covari�ncia local em (k|k)
MATRIX<double> P_bus_global_k(n,n);					// covari�ncia global em (k|k)
MATRIX<double> P_bus_global_k1(n,n);				// covari�ncia global em (k|k-1)
MATRIX<double> E_bus(n,n);						// covari�ncia do erro
MATRIX<double> K_bus(n,m_bus);					// ganho do EKF
MATRIX<double> H_bus(m_bus,n);					// Jacobiano de h(x)
MATRIX<double> R_bus(m_bus,m_bus);					// vari�ncias da medida






