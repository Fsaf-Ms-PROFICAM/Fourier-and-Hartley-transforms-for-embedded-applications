/*
	Programa de testes processamento decomposição de sinal em espectro de frequências.
	TCC - Junio Cesar Ferreira - 2016 Unifran
	Teste Algoritmo Cooley-Tukey Radix 2 para Transformada Discreta de Fourier
	------------------------
	Tempos de processamento:
	------------------------
	016 -> 444  us 
	032 -> 680  us
	064 -> 1.18 ms
	128 -> 2.36 ms
	256 -> 4.88 ms
	512 -> 10.4 ms
   1024 -> 22.2 ms
	
*/

#include "tab_trig.h"

// IOs Utitilizados
#define LED   16  
#define Check  5 
#define Teste  4 
 
#define N    16 // Comprimento dos vetores de entrada e saída
#define pi   3.141592653589793

// Vetor de entrada com sinal para simulação
double v[N];
// Vetores de Saída para FFT
double Xreal[N];                // Vetor Componente Real
double Ximag[N];                // Vetor Componente Imaginária
double Wreal[N/2], Wimag[N/2];  // Vetor de coeficientes omega
unsigned ConTimer=0;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * Função Reversora de bits
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
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
			    case 1024:
                        Lim=10;
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

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * FFT Algoritmo Cooley-Tukey radix-2 descimation-in-time
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void FFT(){
    double pi_N=2*pi/N;       // Auxiliar Pi*n
    double aar, aai, bbr, bbi, Ar, Ai, Br, Bi;
    unsigned int i, k, L, desl, P, NP, aux1, aux2;
    // Computa a FFT Cooley-Tukey Radix-2
    yield();
	// Carrega vetores
    for (i=0;i<N;i++){
        Xreal[Table_Reverse(i,N)]=v[i];
        Ximag[i]=0;
    }
    // Carrega vetor de coeficientes
	k=512/N;
    for (i=0;i<N/4;i++){
		if (!i%8) yield();
        //Wreal[i] = cos(pi_N*i);
        //Wimag[i] = -sin(pi_N*i);
		aux1=i*k;
		Wreal[i]    =  tab_trig[i*k];
		Wreal[i+N/4]= -tab_trig[128-aux1];
		Wimag[i]    = -tab_trig[128-aux1];
		Wimag[i+N/4]= -tab_trig[aux1];
    }
	// Computa FFT Cooley-Tukey radix-2
    for (P=N;P>=2;P=P/2){
	   yield();	
       NP = (N/P);    // Comprimento da DFT decomposta
	   L=P/2;         // Limite para deslocamentos da Butterfly do nível atual
       desl = 2*NP;   // Fator de deslocamento interno do nível
	   // Deslocamento da Butterfly principal
       for (k=0;k<NP;k++){
		   aux2=k*P/2;
		   // Deslocamento das Butterflys semelhantes
           for (i=0;i<L;i++){
                // Prepara entradas da Butterfly
				aux1=k+i*desl;
                aar=Xreal[aux1];
                aai=Ximag[aux1];
                bbr=Xreal[NP+aux1];
                bbi=Ximag[NP+aux1];
                // Computa Butterfly DIT ...
				Br=Wreal[aux2];
				Ar=Wimag[aux2];
                Bi=Wimag[aux2];
				Ai=Wreal[aux2];
				Br*=bbr;
				Ar*=bbi;
                Bi*=bbr;
				Ai*=bbi;
				Br-=Ar;
				Bi+=Ai;
                Xreal[aux1]=aar+Br;
                Ximag[aux1]=aai+Bi;
                Xreal[NP+aux1]=aar-Br;
                Ximag[NP+aux1]=aai-Bi;
			}
		}
	}
    // Normaliza resultado
	yield();
	for (i=0;i<N;i++){
        Xreal[i]=sqrt(pow(Xreal[i],2)+pow(Ximag[i],2))/N;
	}
}

void Sample(){
	unsigned int i;
	// Realiza amostragem do sinal
	for (i=0;i<N;i++){
		digitalWrite(Teste,1);
		yield();
		digitalWrite(Teste,0);
		v[i]=(double)analogRead(A0)*3.3/1023;
		delayMicroseconds(585);	// Sample 1KHz
	}
	
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(Check, OUTPUT);
  pinMode(Teste, OUTPUT);
  Serial.begin(115200);
}

void loop() {
    //Sample();
    delay(1);
	digitalWrite(Check, LOW);
	FFT();
	digitalWrite(Check, HIGH);
}
