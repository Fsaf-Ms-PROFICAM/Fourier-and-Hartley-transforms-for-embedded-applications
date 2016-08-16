/*
    Programa de testes Transformada Discreta de Fourier
	Algoritmo DFT baseado no somatório
    Desenvolvido por Junio Cesar Ferreira
    Data: 29/12/2015
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N    128  // Comprimento dos vetores de entrada e saída
#define pi   3.1415926535897932384626433832795028841971693993751

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

	double Xreal[N]; // Vetor Saída Real
	double Ximag[N]; // Vetor Saída Imaginária
	double pi_N;     // Auxiliar Pi*n
	unsigned int i;  // Indexador de uso geral
	printf("\n * Transformada Discreta de Fourier * "); 
	// Calculo da DFT
	for (int k=0; k<N; k++){
		Xreal[k] = 0;
		Ximag[k] = 0;
		pi_N = 2*k*pi;
		for (int n=0; n<N; n++){
			Xreal[k] += x[n]*cos((pi_N*n)/N);
			Ximag[k] -= x[n]*sin((pi_N*n)/N);
		}
		Xreal[k]=2*Xreal[k]/N;
		Ximag[k]=2*Ximag[k]/N;
	}
	// Exibe vetor de entrada
	printf("\n\nEntrada:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", x[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", x[i]);
    // Exibe resultado em complexos
	printf("\n\nResultado DFT:\n");
	for (i=0;i<N/4;i++){
		if (Ximag[i]>=0) printf(" %.2f+%.2fi ", Xreal[i],Ximag[i]);
		else printf(" %.2f%.2fi ", Xreal[i],Ximag[i]);
	} 
	printf("\n");
	for (;i<N/2;i++){
		if (Ximag[i]>=0) printf(" %.2f+%.2fi ", Xreal[i],Ximag[i]);
		else printf(" %.2f%.2fi ", Xreal[i],Ximag[i]);
	}
	printf("\n");
	for (;i<3*N/4;i++){
		if (Ximag[i]>=0) printf(" %.2f+%.2fi ", Xreal[i],Ximag[i]);
		else printf(" %.2f%.2fi ", Xreal[i],Ximag[i]);
	}
	printf("\n");
	for (;i<N;i++){
		if (Ximag[i]>=0) printf(" %.2f+%.2fi ", Xreal[i],Ximag[i]);
		else printf(" %.2f%.2fi ", Xreal[i],Ximag[i]);
	}
    // Exibe módulo do resultado 
	printf("\n\nModulo da saida:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", sqrt(pow(Xreal[i],2)+pow(Ximag[i],2)));
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", sqrt(pow(Xreal[i],2)+pow(Ximag[i],2)));
	
	//printf("\n\nHartley por Fourier:\n");
	//for (i=0;i<N/2;i++)  printf(" %.4f ", sqrt(pow(Xreal[i]-Ximag[i],2)));
	//printf("\n");
	//for (;i<N;i++) printf(" %.4f ", sqrt(pow(Xreal[i]-Ximag[i],2)));
	
 
}
