/*
	Programa de testes processamento decomposição de sinal em espectro de frequências.
	Implementação DHT com ESP8266 e DFT via DHT
	TCC - Junio Cesar Ferreira - 2016 Unifran
*/

// IOs Utilizados
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

  double v[N];            // Vetor de entrada
  double V[N];            // Vetor de saída
  double pi_N=2*pi/N;     // Auxiliar Pi*n
  double wre, wro;        // Componentes Pares e impares
  unsigned int i;         // Indexador de uso geral
  unsigned int n, k;      // Indexadores das transformadas

void loop() {   
  // Realiza amostragem do sinal 
  for (i=0;i<N;i++){
	digitalWrite(Teste,1);
    yield();
	digitalWrite(Teste,0);
    v[i]=(double)analogRead(A0)*3.3/1023;
	delayMicroseconds(585); // Sample 1KHz
	//delayMicroseconds(80); // Sample 2KHz	
  }

  digitalWrite(Check,HIGH); // Ponto de checagem
  // Computa DHT
  for (k=0;k<N/2;k++){
    V[k]=0;
    V[k+N/2]=0;
    yield();
	for (n=0;n<N/2;n++){
      wre = cos(pi_N*2*n*k-pi/4);
      wro = cos(pi_N*k*(2*n+1)-pi/4);
      V[k]+=(wre*v[2*n]+wro*v[2*n+1]);
      V[k+N/2]+=(wre*v[2*n]-wro*v[2*n+1]);
	}
	V[k]=2*V[k]/N;
	V[k+N/2]=2*V[k+N/2]/N;
  }
  yield();
  // Computa T. de Fourier atraves da T. Hartley
  for (i=1;i<N/2;i++){
      V[i]=sqrt(V[i]*V[i]+V[N-i]*V[N-i]);
      //V[N-i]=V[i];
  }
  digitalWrite(Check,LOW);
  
  digitalWrite(LED, LOW);
  // Envia resultados via Serial
  Serial.print("E:");
  for (i=0;i<N;i++){
    Serial.print('[');
    Serial.print(v[i]);
	Serial.print(']');
  }
  Serial.print(';');
  
  delay(10);
  
  Serial.print("S:");
  for (i=0;i<N/2;i++){
    Serial.print('[');
    Serial.print(V[i]);
	Serial.print(']');
  }
  Serial.println(';');
  
  digitalWrite(LED, HIGH);

}
