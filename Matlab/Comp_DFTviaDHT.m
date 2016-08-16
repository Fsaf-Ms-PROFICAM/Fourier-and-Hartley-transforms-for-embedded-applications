% TCC - Desempenho comparativo entre as transformadas de Hartley e Fourier
% Desenvolvido por Junio Cesar Ferreira
% 30/05/2016

clc;
clear all;

% Parâmetros gerais
Fs = 2000;                    % Sampling frequency
T = 1/Fs;                     % Sample time
N = 128;                      % Length of signal
t = (0:N-1)*T;                % Time vector
w0= 60*2*pi;                  % signal frequency
fi = 0;%pi/2;                 % phase angle
Amp = 3.2;                    % Amplitude


%x=Amp*(square(w0*t+fi)+1)/2;

%x=x+(sin(600*t+fi)+1)/4;

x=Amp*(sawtooth(-w0*t+fi)+1)/2;

%x = Amp*(sin(w0*t+fi)+1)/2;
%x = Amp*(0.3*cos(w0*t+fi)+0.2*sin(2.4*w0*t+fi)+0.5*cos(3.3*w0*t+pi/4))/2;


% Função sinc resultado da TFC
%syms w;
%w_min=-Fs/2;
%w_max=Fs/2;
%Sinc = Amp*sin(w*(N/Fs)*pi/2)/(w*(N/Fs)*pi/2);


% Plota grafico Sinal amostrado em tempo discreto
figure(1);
subplot(1,2,1);
plot(t*T,x);
title('Sinal');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;
subplot(1,2,2);
stem(t*T, x);
title('Sinal Discreto');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

% Gera espaço de frequências
f = Fs/N*linspace(-((N/2)-1),N/2,N);

%% Transformada Discreta de Fourier
Y = DFT(x,N);                         % Computa DFT
td_Fourier = orgn_espectro(2*abs(Y(1:N)),N);  % Reorganiza vetor 
figure(2);                          
%g=ezplot(abs(Sinc),[w_min,w_max]);    % Plota Res. TDC
%hold on;
%set(g,'Color','red','LineWidth',0.5);
stem(f,td_Fourier);                           % Plota Res. DFT 
%ylim([0,Amp+0.3]);                         
title('DFT');
xlabel('Frequências (Hz)');
ylabel('Amplitude (V)');
grid on;

%% Transformada Discreta de Hartley
Y = DHT(x,N);
td_Hartley = orgn_espectro(2*abs(Y(1:N)),N);
figure(3);
stem(f,td_Hartley);
title('DHT');
xlabel('Frequências (Hz)');
ylabel('Amplitude (V)');
grid on;

%% Computa DFT atraves do resultado da DHT
Hartley_Fourier=1:N;
for i=1:N/2
	Hartley_Fourier(N-i)=(sqrt(2)/2)*sqrt(td_Hartley(i)^2+td_Hartley(N-i)^2);
	Hartley_Fourier(i)=Hartley_Fourier(N-i);
end
Hartley_Fourier(N)=(sqrt(2)/2)*sqrt(td_Hartley(N)^2+td_Hartley(N)^2);

%% Plota res. DFT por DHT
figure(4);
stem(f,Hartley_Fourier);
title('DFT via DHT');
xlabel('Frequências (Hz)');
ylabel('Amplitude (V)');
grid on;

%% Calcula diferença 
Erro_Dif_FH=abs(td_Fourier-Hartley_Fourier);
figure(5);
stem(f,Erro_Dif_FH);
title('Erro=DFT-DHT');
xlabel('Frequências (Hz)');
ylabel('Amplitude (V)');
grid on;

%% Discretiza a função Sinc
%d_sinc=1:N;
%for i=1:N/2
%   x=(i-N/2);
%   d_sinc(N-i)=abs(Amp*sin(x*pi/2)/(x*pi/2));
%   d_sinc(i)=d_sinc(N-i);
%end
%d_sinc(N)=abs(Amp*sin(x*pi/2)/(x*pi/2));

%% Plota diferença entre CFT e DFT
%figure(6);
%stem(f,abs(d_sinc-td_Fourier));
%title('Erro=TF-DFT');
%grid on;

