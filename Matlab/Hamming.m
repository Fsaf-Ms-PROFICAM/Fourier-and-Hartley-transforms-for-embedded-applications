% TCC - Desempenho comparativo entre as transformadas de Hartley e Fourier
% Desenvolvido por Junio Cesar Ferreira
% 08/08/2016

Fs = 2000;                    % Sampling frequency
T = 1/Fs;                     % Sample time
N = 512;                      % Length of signal
t = (0:N-1)*T;                % Time vector
w0= 45*2*pi;
fi = pi/4;
    
x = 3.2*(square(w0*t+fi)+1)/2;

w = 0.54-0.46*cos(t*2*pi/(N*T));

subplot(2,2,1);
plot(t,x);
title('Sinal f(t)');
xlim([0,N*T]);
ylim([0,3.5]);

f = Fs/2*linspace(0,1,N/2+1);

X = DFT(x,N);
A2 = X(1:N/2+1);
subplot(2,2,2);
graf=plot(f,A2);
set(graf,'Color','blue','LineWidth',2);
ylim([0,1]);
title('DFT (F)');
grid on;

for n=1:N
    w(n)=w(n)*x(n);
end

subplot(2,2,3);
plot(t,w);
title('Sinal janelado f(t)w(t) Janela de Hamming');
xlim([0,N*T]);
ylim([0,3.5]);

f = Fs/2*linspace(0,1,N/2+1);

X = DFT(w,N);
A2 = X(1:N/2+1);
subplot(2,2,4);
graf=plot(f,A2);
set(graf,'Color','blue','LineWidth',2);
title('DFT (W*F)');
grid on;
