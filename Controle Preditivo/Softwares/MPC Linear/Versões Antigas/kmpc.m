% Roda programa para gerar as referencias:
oito_ponto;
% reta;

clear all; close all; clc;

T = 0.1; % periodo de amostragem.

N = 3; % horizonte de predicao.

% Ordens do sistema:
n = 3; % do estado.
m = 2; % do controle.

% Matriz linear do sistema:
A = eye(n);

% Matrizes de peso:
Q = eye(n); % do estado.
R = eye(m); % do controle.

% Restricao nos erros do controle:
evlim = inf;
ewlim = inf;
eulim = [evlim ; ewlim];

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

% Construindo as matrizs Abar, Qbar, Rbar e dbar:
Abar = A;
Qbar = Q;
Rbar = R;
edbar = eulim;
for i = 2 : N
    Abar = [Abar ; A^i];
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    edbar = [edbar ; eulim];
end

% Estados de referencia:
load xref;
load yref;
load thetaref;

% Controles de referencia:
load vref;
load wref;

% Inicializacao:
x(1) = -2;
y(1) = 2;
theta(1) = 0;

ex(1) = x(1)-xref(1);
ey(1) = y(1)-yref(1);
etheta(1) = theta(1)-thetaref(1);

% Instante final de simulacao:
Kf = size(vref,2);

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% ESTE EH O LOOP DA TRAJETORIA:
for k = 1 : Kf
    
    % Erro atual:
    % e0 = [x(k)-xref(k) ; y(k)-yref(k) ; theta(k)-thetaref(k)];
    e0 = [ex(k) ; ey(k) ; etheta(k)];
   
    % Construindo a matriz S:
    for col = 1 : N
        for lin = 1 : N
            if col <= lin
                if k+col > Kf
                    B = [T*cos(thetaref(Kf-1)) 0 ; T*sin(thetaref(Kf-1)) 0 ; 0 T];
                else
                    B = [T*cos(thetaref(k+col)) 0 ; T*sin(thetaref(k+col)) 0 ; 0 T];
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = A^(lin-col)*B;
            end
        end
    end
    
    % Construindo a matriz H:
    H = 2*(Rbar + S'*Qbar*S);
    
    % funcao f:
    f = 2*S'*Qbar*Abar*e0;
    
    eu(:,k) = quadprog(H,f,[],[],[],[],-edbar,edbar,[],options);

    % eu e uma matriz de m*N X Kf elementos.
    % As primeiras duas linhas eh o controle do MPC.
    

    u(:,k) = eu(1:2,k) + [vref(k) ; wref(k)];
    
    % Modelo cinematico do erro:
    ex(k+1) = ex(k) + eu(1,k)*cos(thetaref(k))*T;
    ey(k+1) = ey(k) + eu(1,k)*sin(thetaref(k))*T;
    etheta(k+1) = etheta(k) + eu(2,k)*T;
        
    % Modelo cinematico:
    x(k+1) = x(k) + u(1,k)*cos(theta(k))*T;
    y(k+1) = y(k) + u(1,k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + u(2,k)*T;
  
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% FIM DA TRAJETORIA.

disp('Erros finais dos estados:');
disp('em x:');
x(size(x,2))-xref(size(xref,2))
disp('em y:');
y(size(y,2))-yref(size(yref,2))
disp('em theta:');
theta(size(theta,2))-thetaref(size(thetaref,2))

% Graficos:

subplot(2,1,1);
hold on; box on; grid on;
plot(u(1,:),'r'); plot(vref,'-.k');
legend('v','v_{ref}',0);
ylabel('v');
title('Controle');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
plot(u(2,:),'r'); plot(wref,'-.k');
legend('w','w_{ref}',0);
ylabel('w');
hold off;

figure;
subplot(3,1,1);
hold on; box on; grid on;
plot(x,'r'); plot(xref,'-.k');
legend('x','x_{ref}',0);
ylabel('x');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(y,'b'); plot(yref,'-.k');
legend('y','y_{ref}',0);
ylabel('y');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(theta,'g'); plot(thetaref,'-.k');
legend('\theta','\theta_{ref}',0);
ylabel('\theta');
hold off;

figure;
hold on; box on; grid on;
plot(x-xref,'r');
plot(y-yref,'g');
plot(theta-thetaref,'b');
legend('e_x','e_y','e_{\theta}',0);
plot(ex,'-.r'); plot(ey,'-.g'); plot(etheta,'-.b');
hold off;
title('Erros');

figure;
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('traj.','referencia',0);
hold off;
title('Trajetoria');

