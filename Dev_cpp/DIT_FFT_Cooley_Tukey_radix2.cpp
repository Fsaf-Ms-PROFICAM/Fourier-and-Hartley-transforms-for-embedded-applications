/*
    Programa de testes Transformada Discreta de Fourier
	Algoritmo FFT Cooley-Tukey radix-2 Decimation-in-time
    Desenvolvido por Junio Cesar Ferreira
    Data: 19/06/2016
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N    16  // Comprimento dos vetores de entrada e saída
#define pi   3.1415926535897932384626433832795028841971693993751

unsigned int Table_Reverse(unsigned int index, unsigned int Lenght){
	unsigned int mirror, exp, Lim;
	switch (Lenght){
		case 8:
			Lim=3;
			break;
		case 16:
		    Lim=4;
			break;
		case 32:
			Lim=5;
			break;
		case 64:
			Lim=6;
			break;
		case 128:
			Lim=7;
			break;
		case 256:
			Lim=8;
			break;
		case 512:
			Lim=9;
			break;
	}
	mirror = 0;
	for (exp=0;exp<Lim;exp++){
		mirror=mirror<<1;
		mirror+=(0x01&index);
		index=index>>1;
	}
	return mirror;
}


main(){
	double x[N]={1,2,3,1,0,0,0,0,1,2,3,1,0,0,0,0};/*{
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0
	};// Vetor de entrada, inicializado com amostragem de uma onda quadrada
	*/
	double Xreal[N];          // Vetor Saída Real
	double Ximag[N]; 		  // Vetor Saída Imaginária
	double V[N];
	double Wr[N/2], Wi[N/2];  // Vetores de coeficientes 
	double pi_N=2*pi/N;;      // Auxiliar Pi*n
	double ar, ai, br, bi;    
	unsigned int i, k, L, desl, P, NP;
	printf("\n * Transformada Rapida de Fourier * "); 
	// Calculo da FFT
	// Carrega vetores
	for (i=0;i<N;i++){
		Xreal[Table_Reverse(i,N)]=x[i];
		Ximag[i]=0;
	}
    // Carrega vetor de coeficientes
	for (i=0;i<N/2;i++){
		Wr[i] = cos(pi_N*i);
		Wi[i] = -sin(pi_N*i);
		printf("\nWr[%d]=%.3f \tWi[%d]=%.3f",i,Wr[i],i,Wi[i]);
	}

	// Computa FFT Cooley-Tukey radix-2
	for (P=N;P>=2;P=P/2){
		NP = (N/P);  // Comprimento da DFT decomposta
		printf("\nP=%d", P);
		L=P/2;     // Limite para deslocamentos da Butterfly do nível atual
		desl = 2*NP; // Fator de deslocamento interno do nível
		printf("\n L=%d; N/P=%d; desl=%d", L, NP, desl);
		// Deslocamento da Butterfly principal
		for (k=0;k<NP;k++){
			for (i=0;i<L;i++){
				printf("\n i=%d, k=%d",i,k);
				// Prepara entradas da Butterfly
				ar=Xreal[k+i*desl];
				ai=Ximag[k+i*desl];
				br=Xreal[k+NP+i*desl];
				bi=Ximag[k+NP+i*desl];
				// Computa Butterfly DIT ...
				Xreal[k+i*desl]=ar+br*Wr[k*P/2]-bi*Wi[k*P/2];
				Ximag[k+i*desl]=ai+br*Wi[k*P/2]+bi*Wr[k*P/2];
				Xreal[k+NP+i*desl]=ar-br*Wr[k*P/2]+bi*Wi[k*P/2];
				Ximag[k+NP+i*desl]=ai-br*Wi[k*P/2]-bi*Wr[k*P/2];
			}
		}
	}
    // Normaliza resultado
	for (i=0;i<N;i++){
		Xreal[i]=2*Xreal[i]/N;
		Ximag[i]=2*Ximag[i]/N;
		V[i]=sqrt(pow(Xreal[i],2)+pow(Ximag[i],2));
	}
	// Exibe vetor de entrada
	printf("\n\nEntrada:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", x[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", x[i]);
    // Exibe resultado em complexos
	printf("\n\nResultado FFT:\n");
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
	for (i=0;i<N/2;i++) printf(" %.4f ", V[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", V[i]);
	
	//printf("\n\nHartley por Fourier:\n");
	//for (i=0;i<N/2;i++)  printf(" %.4f ", sqrt(pow(Xreal[i]-Ximag[i],2)));
	//printf("\n");
	//for (;i<N;i++) printf(" %.4f ", sqrt(pow(Xreal[i]-Ximag[i],2)));
	
 
}
