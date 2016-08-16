/*
    Programa de testes Transformada Discreta de Hartley
	Algoritmo DHT baseado no somatório
    Desenvolvido por Junio Cesar Ferreira
    Data: 29/12/2015
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N    128  // Comprimento dos vetores de entrada e saída
#define pi   3.1415926535897932384626433832795028841971693993751

// Módulo de um valor
double Modulo(double Val){
	if (Val<0) return Val*-1;
	else return Val;
}

main(){
	double x[N]=
	{
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0
	};// Vetor de entrada, inicializado com amostragem de uma onda quadrada
	double X[N];     // Vetor Saída 
	double pi_N;     // Auxiliar Pi*n
	unsigned int i;  // Indexador de uso geral
	printf("\n * Transformada Discreta de Hartley * ");  
	// Calculo da DHT
	for (int n=0; n<N; n++){
		X[n] = 0;
		pi_N = 2*n*pi;
		for (int k=0; k<N; k++){
			X[n] += x[k]*sqrt(2)*(cos((pi_N*k)/N-pi/4));//+sin((pi_N*k)/N));
		}
		X[n]=2*X[n]/N;
	}
	// Exibe vetor de entrada
	printf("\n\nEntrada:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", x[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", x[i]);
    // Exibe resultado
	printf("\n\nResultado DHT:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", X[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", X[i]);
	// Valor absoluto
	printf("\n\nEm Modulo:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", Modulo(X[i]));
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", Modulo(X[i]));
}
