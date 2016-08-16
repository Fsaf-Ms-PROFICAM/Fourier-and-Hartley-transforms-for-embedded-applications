% TCC - Desempenho comparativo entre as transformadas de Hartley e Fourier
% Função Transformada Discreta de Hartley
% Desenvolvido por Junio Cesar Ferreira
% 26/05/2016
function [H] = DHT(x,N)
  for n=0:N-1
	H(n+1) = 0;
	pi_N = 2*n*pi/N;
	for k=0:N-1
		H(n+1) = H(n+1)+x(k+1)*(cos(pi_N*k)+sin(pi_N*k));
    end
	H(n+1)=2*abs(H(n+1))/N;
  end
  H=H/2;
end
