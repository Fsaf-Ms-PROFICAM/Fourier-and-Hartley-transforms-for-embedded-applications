% Plota gráfico da transformada de Fourier de um sinal quadrado
% Desenvolvido por Junio Cesar Ferreira
% 25/07/2016

clc;
clear all;

T=1/62.5; % Periodo
A=3.2;  % Amplitude

nw=(0:62.5:500);
mnw=(-437.5:62.5:-62.5);
w=(-500:0.1:500);
F=A/2*sinc(w/2*T);
g=plot(w,abs(F),'--');
set(g,'Color','red','LineWidth',1.5);
set(gca,'XTick',-437.5:62.5:437.5);
xlabel('Frequência Hz');
ylabel('Amplitude V');
hold on;
D=A/2*sinc(nw/2*T)
mD=A/2*sinc(mnw/2*T);
stem(mnw,abs(mD));
hold on;
stem(nw,abs(D));
