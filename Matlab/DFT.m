% TCC - Desempenho comparativo entre as transformadas de Hartley e Fourier
% Função Transformada Discreta de Fourier
% Desenvolvido por Junio Cesar Ferreira
% 26/05/2016
function [F] = DFT(x,N)
  for k=0:N-1
	Xreal(k+1) = 0;
	Ximag(k+1) = 0;
	pi_N = 2*k*pi/N;
	for n=0:N-1
		Xreal(k+1) = Xreal(k+1)+ x(n+1)*cos(pi_N*n);
		Ximag(k+1) = Ximag(k+1)- x(n+1)*sin(pi_N*n);
    end
	Xreal(k+1)=2*Xreal(k+1)/N;
	Ximag(k+1)=2*Ximag(k+1)/N;
  end
  for k=1:N
      F(k)=sqrt(Xreal(k)^2+Ximag(k)^2);
  end
  F=F/2;
end
