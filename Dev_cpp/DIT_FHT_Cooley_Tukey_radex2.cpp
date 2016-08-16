/*
    Programa de testes Transformada Discreta de Hartley
	Algoritmo FHT Cooley-Tukey radix-2 decimation-in-time
    Desenvolvido por Junio Cesar Ferreira
    Data: 25/06/2016
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define N    16  // Comprimento dos vetores de entrada e saída
#define pi   3.1415926535897932384626433832795028841971693993751

double Modulo(double Val){
	if (Val<0) return Val*-1;
	else return Val;
}

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

unsigned int Complemento(unsigned int K, unsigned int L){
	if (K==0) return 0;
	else return (L-K);
}

main(){
	double x[N]={1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0};
	/*
	{
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
	double X[N];
	double C[N/2],S[N/2];
	double pi_N=2*pi/N;      // Auxiliar Pi*n
	double a, b, c;
	unsigned int i, k, L, desl, P, NP;
	printf("\n * Transformada Rapida de Hartley * \n"); 
	// Calculo da FHT
	// Carrega vetores
	for (i=0;i<N;i++) X[Table_Reverse(i,N)]=x[i];

    // Carrega vetor de coeficientes
	for (i=0;i<N/2;i++){
		C[i]=cos(i*2*pi/N);
		S[i]=sin(i*2*pi/N);
		printf("i=%d\tC=%.4f\tS=%.4f\n", i, C[i], S[i]);
	}

	// Computa FHT Cooley-Tukey radix-2
	for (P=N;P>=2;P=P/2){
		NP = (N/P);  // Comprimento da DHT decomposta
		printf("\nP=%d", P);
		L=P/2;          // Limite para deslocamentos da Butterfly do nível atual
		desl = 2*NP;    // Fator de deslocamento interno do nível
		printf("\nL=%d; N/P=%d; desl=%d\n", L, NP, desl);
		// Deslocamento da Butterfly principal
		volatile double R[N];
		for (k=0;k<NP;k++){ // Desloca grupo de butterflys
			for (i=0;i<L;i++){ // Desloca Butterfly
				printf("\n i=%d\tk=%d\t-k=%d",i,k,Complemento(k,NP));
				// Prepara entradas da Butterfly
				a=X[k+i*desl];
				b=X[k+NP+i*desl]*C[k*P/2];
				c=X[Complemento(k,NP)+NP+i*desl]*S[k*P/2];
				// Computa Butterfly DIT ...
				R[k+i*desl]=a+b+c;
				R[k+NP+i*desl]=a-b-c;
			}
		}
		printf("\n");
		for (i=0;i<N;i++) X[i]=R[i];
	}
    // Normaliza resultado
	for (i=0;i<N;i++) X[i]=2*X[i]/N;
	
	// Exibe vetor de entrada
	printf("\n\nEntrada:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", x[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", x[i]);
    // Exibe resultado
	printf("\n\nResultado FHT:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", X[i]);
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", X[i]);
	// Valor absoluto
	printf("\n\nEm Modulo:\n");
	for (i=0;i<N/2;i++) printf(" %.4f ", Modulo(X[i]));
	printf("\n");
	for (;i<N;i++) printf(" %.4f ", Modulo(X[i]));
}

