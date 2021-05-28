function ubar = uquadprog(A,B,Q,R,N,x0,xrbar,L,d)

%UQUADPROG realiza a minimizacao de uma sequencia de controle.
%	ubar = uquadprog(A,B,Q,R,N,x0,xrbar,L,d)
% minimiza uma sequencia de controle de tamanho N para o sistema
%    x(k+1|k) = Ax(k|k) + Bu(k|k)
% para uma funcao de custo quadratico
%            min 0.5*u'*H*u + f'*u 
%             u    
% sujeita a restricoes do tipo L*u <= b.
%
% Entradas:
%  A, B:     matrizes do sistema.
%  Q:        matriz de peso do erro e.
%  R:        matriz de peso do controle u.
%  N:        horizonte de predicao.
%  x0:       estado no instante atual.
%  xrbar:    vetor de referencia aumentado de dimensao N.
%  L:        matriz de peso da restricao.
%  d:        valor limite do controle.
%
% Saida: sequencia otimizada de controle ubar, na forma
%        de um vetor de N*size(u) elementos.

if nargin < 9,
    error('UQUADPROG precisa de 9 argumentos de entrada.')
end

% Construindo as matrizs Abar, Qbar e Rbar:
Abar = A;
Qbar = Q;
Rbar = R;
for lin = 2 : N
    Abar = [Abar ; A^lin];
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
end

% Construindo a matriz S:
for col = 1 : N
    for lin = 1 : N
        if col <= lin
            S(lin*size(B,1)-size(B,1)+1:lin*size(B,1),col*size(B,2)-size(B,2)+1:col*size(B,2)) = A^(lin-col)*B;
        end
    end
end

H = 2*(Rbar+S'*Qbar*S);
f = 2*S'*Qbar*(Abar*x0-xrbar);

ubar = quadprog(H,f,L,b);
