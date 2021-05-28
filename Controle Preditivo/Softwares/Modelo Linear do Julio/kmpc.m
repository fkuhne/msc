clear all; close all; clc;

% Inicializacao das constantes:
T = 0.1; % periodo de amostragem.
v = 30; % velocidade tangencial.
N = 5; % horizonte de predicao.

% Modelo linear do Julio:
% x(k+1) = Ax(k) + Bu(k)
A = eye(2);
B = [(v*T)^2/2 ; v*T];

% Matrizes de peso:
Q = eye(size(A)); % do estado,
R = eye(size(B,2)); % do controle

% Restricao no controle:
ulim = 1.9;
d = ones(N*size(B,2),1);
dtil = ulim*d;

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

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

% Construindo a matriz H:
H = 2*(Rbar+S'*Qbar*S);

% Inicializacao:
y(1) = 0;
theta(1) = 0;

% Trajetoria de referencia (no sistema global):
load yref;
load thetaref;

% ESTE EH O LOOP DA TRAJETORIA:
for k = 1 : size(yref,2)-1
    
    x0 = [y(k) ; theta(k)]; % posicao atual.
    
    p=0;
    
    % Construindo o vetor xrbar:
    xrefbar = [yref(k+1) ; thetaref(k+1)];
    for i = 2 : N
        if k+i > size(yref,2)-1
            p=p+1
            xrefbar = [xrefbar ; yref(k) ; thetaref(k)];
        else
            xrefbar = [xrefbar ; yref(k+i) ; thetaref(k+i)];        
        end
    end
  
    % funcao f:
    f = 2*S'*Qbar*(Abar*x0 - xrefbar);
    
    gamma(:,k) = quadprog(H,f,[],[],[],[],-dtil,dtil,[],options);
    % gamma e uma matriz de N X size(y,2) elementos.
    % A primeira linha eh o controle do MPC.
    
    % Modelo linear do Julio:
    y(k+1) = y(k) + ((v*T)^2/2)*gamma(1,k);
    theta(k+1) = theta(k) + v*T*gamma(1,k);
 
end

load gammaref;

% Graficos:
subplot(3,1,1);
hold on; box on; grid on;
plot(1:size(y,2), y, 'r');
plot(1:size(yref,2), yref, '-.k');
plot(1:size(theta,2), theta);
plot(1:size(thetaref,2), thetaref, '-.k');
title('Estados y e \theta');
legend('y','y_{ref}','\theta','\theta_{ref}',0);
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(1:size(gamma,2), gamma(1,:),'r');
plot(1:size(gammaref,2), gammaref,'-.k');
title('Controle \gamma');
legend('\gamma','\gamma_{ref}',0);
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(1:size(y,2), y(1,:)-yref,'r');
plot(1:size(theta,2), theta(1,:)-thetaref,'b');
title('Erros dos estados');
legend('e_y','e_{\theta}',0);
hold off;

