/*
	Analisador de vibrações mecânicas baseado no acelerômetro MPU6050 e ESP8266
	Desenvolvido por Junio Cesar Ferreira - e-mail: jcf_ssp@hotmail.com
	TCC - Engenharia Mecatrônica - Unifran - 2016
*/

#include <Wire.h>
#include "tab_trig.h"

// IOs utilizados
#define 	LED    		16
#define 	LED_CONFIG 	5
#define 	Teste   	4
#define		Botao		0

// Constantes e parâmetros da FHT
#define 	N     		512         // Comprimento dos vetores   
#define 	pi    		3.141592653 // Constante pi
#define 	r2s2  		0.707106781 // Raiz de dois sobre dois

// Constante de deslocamento longitudinal sobre cada eixo, valores calibrados em bancada
#define     Desl_X      0
#define     Desl_Y      27
#define    	Desl_Z      -160

// Constantes de indexação dos eixos
#define 	X			0
#define 	Y			1
#define 	Z			2

// Endereço I2C do MPU-6050
#define 	   MPU   		      0x68
// Endereçamento registros do MPU6050
#define        SMPLRT_DIV         0x19
#define        CONFIG             0x1A
#define        GYRO_CONFIG        0x1B
#define        ACCEL_CONFIG       0x1C
#define        ACCEL_XOUT_H       0x3B
#define        ACCEL_XOUT_L       0x3C
#define        ACCEL_YOUT_H       0x3D
#define        ACCEL_YOUT_L       0x3E
#define        ACCEL_ZOUT_H       0x3F
#define        ACCEL_ZOUT_L       0x40
#define        TEMP_OUT_H         0x41
#define        TEMP_OUT_L         0x42
#define        GYRO_XOUT_H        0x43
#define        GYRO_XOUT_L        0x44
#define        GYRO_YOUT_H        0x45
#define        GYRO_YOUT_L        0x46
#define        GYRO_ZOUT_H        0x47
#define        GYRO_ZOUT_L        0x48
#define        PWR_MGMT_1         0x6B
#define        WHO_AM_I           0x75
#define        SlaveAddress       0xD0

// Fator de conversão para mm/s^2
#define        Fator_Conv         4.8  

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// Variaveis globais
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Vetor de entrada dos três eixos do acelerômetro
double   v[3][N];
// Vetor de saída com espectro de Fourier
double   Fourier[N];    
// Vetor auxiliar Transformada de Hartley       
double   Hartley[N];
// Vetores de coeficientes trigonometricos
double   C[N/2],S[N/2];
// Contador de tempo do programa
unsigned ConTimer=0;
// Variaveis de leitura do acelerômetro MPU-6050
int      AcX,AcY,AcZ;
// Offsets Acelerometro
int 	 OfX,OfY,OfZ; 
// Variaveis de controle
bool	 Run=false;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// Protótipos de funções
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
unsigned int Complemento(unsigned int K,unsigned int L);
double       Modulo(double Val);
unsigned int Table_Reverse(unsigned int index, unsigned int Lenght);
void         FHT(unsigned short E);
void         Sample();
void 		 WriteRegMPU(char Adr, char Val);
signed int   GetAcel();
void 		 Send_Data(char type);
void 		 MPU_OffSet();

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// SETUP
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void setup(){
  // Configura Saídas digitais
  pinMode (LED,        OUTPUT);  
  pinMode (LED_CONFIG, OUTPUT);
  pinMode (Teste,      OUTPUT);
  pinMode (Botao,       INPUT);
  // Sinaliza inicialização
  digitalWrite(LED_CONFIG,HIGH);
  // Inicializa I2C para MPU-6050
  Wire.begin(14, 12);         // SDA, SCL
  WriteRegMPU(PWR_MGMT_1,   0x00);
  //WriteRegMPU(SMPLRT_DIV,   0x07);
  WriteRegMPU(CONFIG,       0x00);
  //WriteRegMPU(GYRO_CONFIG,  0x18);
  WriteRegMPU(ACCEL_CONFIG, 0x19);
  // UART
  Serial.begin(115200);       // Configura canal Serial 
  delay(200);
  digitalWrite(LED_CONFIG,LOW);
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// 	LOOP
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void loop(){
    if (ConTimer>25){
		digitalWrite(LED, LOW);
		// Faz amostragem dos sinais do acelerômetro
		Sample();
		Send_Data('d');
		// Computa FFT via FHT
		FHT(X);FHT(Y);FHT(Z);
		Send_Data('f');
		digitalWrite(LED, HIGH);
		ConTimer=0;
	}else{
		delay(100);
		ConTimer++;
	}
	if (!digitalRead(Botao)){
		digitalWrite(LED_CONFIG, HIGH);
		MPU_OffSet();	
		digitalWrite(LED_CONFIG,LOW);  
		
	}

}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//  Função Offset
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void MPU_OffSet(){
	signed long AcumX=0, AcumY=0, AcumZ=0;
	unsigned short i;
	
	for (i=0;i<128;i++){
		Wire.beginTransmission(MPU);     // Inicia comunicação com MPU
		Wire.write(0x3B);  				 // Começa leitura pelo endereço 0x3B (ACCEL_XOUT_H)
		Wire.endTransmission(false);         
		Wire.requestFrom(MPU,6,1);       // Faz requisição para leitura de 6 registros
		AcumX+=GetAcel();
		AcumY+=GetAcel();
		AcumZ+=GetAcel();
		delay(3);
	}
	OfX=AcumX/128; 
	OfY=AcumY/128;  
	OfZ=AcumZ/128;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Função de envio de dados via Serial
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void Send_Data(char type){
	unsigned int i;
	unsigned short j;
	switch (type){
		case 'd':
			// Envia resultados via Serial
			for (j=0;j<3;j++){
				switch (j){
					case 0:
						Serial.print("x:");
						break;
					case 1:
						Serial.print("y:");
						break;
					case 2:
						Serial.print("z:");
						break;
				}
				yield();
				for (i=0;i<N;i++){  
					Serial.print('[');
					Serial.print(v[j][i]);
					Serial.print(']');
				}
				Serial.print(';');
			}
			break;
		case 'f':
			// Envia resultados via Serial
			for (j=0;j<3;j++){
				switch (j){
					case 0:
						Serial.print("X:");
						break;
					case 1:
						Serial.print("Y:");
						break;
					case 2:
						Serial.print("Z:");
						break;
				}
				yield();
				for (i=0;i<N/2;i++){  
					Serial.print('[');
					Serial.print(v[j][i]);
					Serial.print(']');
				}
				Serial.print(';');
			}
			break;
	}
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Função de escrita nos registros do MPU6050
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void WriteRegMPU(char Adr, char Val){
	Wire.beginTransmission(MPU);// Inicializa comunicação
	Wire.write(Adr);            // Endereço do registro a ser escrito 
	Wire.write(Val);            // Valor a ser escrito 
	Wire.endTransmission(true); // Sinaliza fim da transmissão
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Função de leitura dos dados do acelerômetro
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
signed int GetAcel(){
	int16_t B; 
	uint8_t H,L;
	H=Wire.read();
	L=Wire.read();
	B=(H<<8)+L;
	return B;
}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Função que faz a amostragem
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void Sample(){
	 unsigned short i;
	 for (i=0;i<N;i++){
		if (i%16==0) yield();              
		digitalWrite(Teste, LOW);    // Ponto de teste  
		Wire.beginTransmission(MPU); // Inicia comunicação com MPU
		Wire.write(0x3B);  			 // Começa leitura pelo endereço 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);         
        Wire.requestFrom(MPU,6,1);   // Faz requisição para leitura de 6 registros
        digitalWrite(Teste, HIGH);
		AcX=GetAcel();               // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
        AcY=GetAcel();               // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ=GetAcel();               // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		v[X][i]=(double)(AcX-OfX);
		v[Y][i]=(double)(AcY-OfY);
		v[Z][i]=(double)(AcZ-OfZ);
	 }
	 for (i=0;i<N;i++){
		 v[X][i]=v[X][i]*Fator_Conv;  // 10 ~= 9.8 m/s^2
		 v[Y][i]=v[Y][i]*Fator_Conv;
		 v[Z][i]=v[Z][i]*Fator_Conv;
	 }
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Função complemento do indexador
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
unsigned int Complemento(unsigned int K, unsigned int L){
	if (K==0) return 0;
	else return (L-K);
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Função valor absoluto para double
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
double Modulo(double Val){
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
void FHT(unsigned short E){
	double a, b, c;
	unsigned int i, k, L, desl, P, NP, aux1, aux2, aux3;
	
	yield();
	// Janela de Hamming
	for (i=0;i<N;i++) Hartley[i]=(0.54-0.46*cos(i*2*pi/N))*v[E][i];
	yield();
	// Carrega vetores
	for (i=0;i<N;i++) Fourier[Table_Reverse(i,N)]=Hartley[i];

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
				a=Fourier[aux1];
				b=Fourier[NP+aux1];
				b*=C[aux3];
				c=Fourier[aux2+aux1];
				c*=S[aux3];
				// Computa Butterfly DIT ...
				Hartley[aux1]=a+b+c;
				Hartley[NP+aux1]=a-b-c;
			}
		}
		for (i=0;i<N;i++) Fourier[i]=Hartley[i];
	}
    // Normaliza resultado
	yield();
	// Computa T. de Fourier atraves da T. Hartley
	for (i=0;i<N/2;i++){
		v[E][i]=r2s2*sqrt(Fourier[i]*Fourier[i]+Fourier[N-i]*Fourier[N-i])/N;
	}
}

/*!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$
 01000101 01110011 01110100 01100101 01110011 00100000 01100011 11110011 01100100
 01101001 01100111 01101111 01110011 00100000 01100110 01101111 01110010 01100001
 01101101 00100000 01100100 01100101 01110011 01100101 01101110 01110110 01101111
 01101100 01110110 01101001 01100100 01101111 01110011 00100000 01110000 01101111
 01110010 00100000 01001010 01110101 01101110 01101001 01101111 00100000 01000011
 01100101 01110011 01100001 01110010 00100000 01000110 01100101 01110010 01110010
 01100101 01101001 01110010 01100001
!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$!@#$*/
