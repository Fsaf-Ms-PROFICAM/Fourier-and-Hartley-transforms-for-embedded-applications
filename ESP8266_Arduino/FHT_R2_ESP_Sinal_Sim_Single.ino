/*
	Programa de testes processamento decomposição de sinal em espectro de frequências.
	TCC - Junio Cesar Ferreira - 2016 Unifran
	Teste Algoritmo Cooley-Tukey Radix 2 para Transformada Discreta de Hartley
*/

#include "tab_trig.h"

// IOs Utilizados
#define LED   16
#define Check  5 
#define Teste  4

#define N    512  // Comprimento dos vetores de entrada e saída
#define pi   3.141592653589793
#define r2s2 0.707106781

// Vetor de entrada com sinal para simulação
float v[N]={
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,
};
// Vetor de saída com espectro de Fourier
float X[N];            
float Ha[N];
float C[N/2],S[N/2];
unsigned ConTimer=0;

// Função complemento do indexador
unsigned int Complemento(unsigned int K, unsigned int L){
	if (K==0) return 0;
	else return (L-K);
}

// Função valor absoluto para float
float Modulo(float Val){
	if (Val<0) return Val*-1;
	else return Val;
}
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
// * FHT Algoritmo Cooley-Tukey radix-2 descimation-in-time
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void FHT(){
	float a, b, c;
	unsigned int i, k, L, desl, P, NP, aux1, aux2, aux3;
	
	yield();
	// Carrega vetores
	for (i=0;i<N;i++) X[Table_Reverse(i,N)]=v[i];

    // Carrega vetor de coeficientes
	k=512/N;
	for (i=0;i<N/4;i++){
		if (!i%8) yield();
		aux1=i*k;
		C[i]    =  tab_trig[aux1];
		C[i+N/4]= -tab_trig[128-aux1];
		S[i]    =  tab_trig[128-aux1];
		S[i+N/4]=  tab_trig[aux1];
	}

	// Computa FHT Cooley-Tukey radix-2
	for (P=N;P>=2;P=P/2){
		yield();
		NP = (N/P);  // Comprimento da DHT decomposta
		L=P/2;          // Limite para deslocamentos da Butterfly do nível atual
		desl = 2*NP;    // Fator de deslocamento interno do nível
		// Deslocamento da Butterfly principal
		for (k=0;k<NP;k++){ // Desloca grupo de butterflys
		    aux2=Complemento(k,NP)+NP-k;
			aux3=k*P/2;
			for (i=0;i<L;i++){ // Desloca Butterfly
				aux1=k+i*desl;
				// Prepara entradas da Butterfly
				a=X[aux1];
				b=X[NP+aux1];
				b*=C[aux3];
				c=X[aux2+aux1];
				c*=S[aux3];
				// Computa Butterfly DIT ...
				Ha[aux1]=a+b+c;
				Ha[NP+aux1]=a-b-c;
			}
		}
		for (i=0;i<N;i++) X[i]=Ha[i];
	}
    // Normaliza resultado
	yield();
	// Computa T. de Fourier atraves da T. Hartley
	for (i=0;i<N/2;i++){
		X[i]=r2s2*sqrt(X[i]*X[i]+X[N-i]*X[N-i])/N;
	}
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(Check, OUTPUT);
  pinMode(Teste, OUTPUT);
  Serial.begin(115200);
}

void loop() {
    unsigned int i;       
  
  	if (ConTimer>50){
		digitalWrite(LED, LOW);
		FHT();
		// Envia resultados via Serial
		Serial.print("E:");
		yield();
		for (i=0;i<N;i++){  
			Serial.print('[');
			Serial.print(v[i]);
			Serial.print(']');
		}
		Serial.print(';');
		Serial.print("S:");
		yield();
		for (i=0;i<N/2;i++){  
			Serial.print('[');
			Serial.print(X[i]);
			Serial.print(']');
		}
		Serial.println(';');
		digitalWrite(LED, HIGH);
		ConTimer=0;
	}else{
		delay(100);
		ConTimer++;
	}

}


