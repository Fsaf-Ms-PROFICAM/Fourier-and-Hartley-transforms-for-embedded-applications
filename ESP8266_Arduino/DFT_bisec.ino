/*
	Programa de testes processamento decomposição de sinal em espectro de frequências.
	Uso da transformada discreta de Fourier, com ordem de complexidade O(N²).
	TCC - Junio Cesar Ferreira - 2016 Unifran
*/

// IOs Utitilizados
#define LED   16  
#define Check  5 
#define Teste  4 
 
#define N    192  // Comprimento dos vetores de entrada e saída
#define pi   3.141592653589793


void setup() {
  pinMode(LED, OUTPUT);
  pinMode(Check, OUTPUT);
  pinMode(Teste, OUTPUT);
  Serial.begin(115200);
}

  double v[N];              // Vetor de entrada
  double Vreal[N];          // Vetor Saída Real
  double Vimag[N];          // Vetor Saída Imaginária
  double pi_N=2*pi/N;       // Auxiliar Pi*n
  double wre, wie, wro, wio;// Componentes da transformada
  unsigned int i;           // Indexador de uso geral
  unsigned int n, k;        // Indexadores da transformada

void loop() {

  // Realiza amostragem do sinal
  for (i=0;i<N;i++){
	digitalWrite(Teste,1);
    yield();
	digitalWrite(Teste,0);
    v[i]=(double)analogRead(A0)*3.3/1023;
	delayMicroseconds(585);	// Sample 1KHz
  }

  digitalWrite(Check,HIGH); // Ponto de checagem 
  // Computa a DFT partes Par e Impar
  for (k=0;k<N/2;k++){
    Vreal[k]=0;
    Vimag[k]=0;
    Vreal[k+N/2]=0;
    Vimag[k+N/2]=0;
	yield();
    for (n=0;n<N/2;n++){
      wre = cos(pi_N*2*n*k);
      wie = sin(pi_N*2*n*k);
      wro = cos(pi_N*k*(2*n+1));
      wio = sin(pi_N*k*(2*n+1));
      Vreal[k]+=(wre*v[2*n]+wro*v[2*n+1]);
      //Vreal[k+N/2]+=(wre*v[2*n]-wro*v[2*n+1]);
      Vimag[k]-=(wie*v[2*n]+wio*v[2*n+1]);
      //Vimag[k+N/2]-=(wie*v[2*n]-wio*v[2*n+1]);
    }
    Vreal[k]=2*Vreal[k]/N;
    Vimag[k]=2*Vimag[k]/N;
    //Vreal[k+N/2]=Vreal[k+N/2]/N;
    //Vimag[k+N/2]=Vimag[k+N/2]/N;
  }

  yield();
  // Computa módulo da DFT
  for (i=1;i<N/2;i++){
      Vreal[i]=sqrt(Vreal[i]*Vreal[i]+Vimag[i]*Vimag[i]);
	  //Vreal[N-i]=Vreal[i];
  }
  digitalWrite(Check,LOW);
  
  digitalWrite(LED, LOW);
  // Envia resultados via Serial
  Serial.print("E:");
  for (i=0;i<N;i++){
	yield();  
    Serial.print('[');
    Serial.print(v[i]);
	Serial.print(']');
  }
  Serial.print(';');
  
  delay(10);
  yield();
  Serial.print("S:");
  for (i=0;i<N/2;i++){  
    yield();
    Serial.print('[');
    Serial.print(v[i]);
	Serial.print(']');
  }
  Serial.println(';');
  
  digitalWrite(LED, HIGH);

}
