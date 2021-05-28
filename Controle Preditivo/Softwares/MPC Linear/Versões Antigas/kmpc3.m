modelo2;

clear all; close all; clc;

T = 0.1; % periodo de amostragem.

N = 5; % horizonte de predicao.

% Ordens do sistema:
n = 3; % do estado.
m = 2; % do controle.

A = eye(n);

% Matrizes de peso:
Q = eye(n); % do erro do estado.
R = eye(m); % do controle.
P = eye(m); % do erro do controle.

% Restricao nos erros do controle:
vlim = inf;
wlim = inf;
ulim = [vlim ; wlim];

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

% Construindo as matrizs Qbar e Rbar e o vetor bar:
Qbar = Q;
Rbar = R;
Pbar = P;
Abar = A;
dbar = ulim;
for i = 2 : N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    Pbar = blkdiag(Pbar,P);    
    Abar = [Abar ; A];
    dbar = [dbar ; ulim];
end

% Estados de referencia:
load xref;
load yref;
load thetaref;

% Controles de referencia:
load vref;
load wref;

% Instante final de simulacao:
Kf = size(vref,2);

% Inicializacao:
x(1) = 0;
y(1) = 0;
theta(1) = 0;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% ESTE EH O LOOP DA TRAJETORIA:
for k = 1 : Kf
    
    % Estado atual:
    q0 = [x(k) ; y(k) ; theta(k)];
    
    % Matriz A(k) atual:
%     A = [1 0 -vref(k)*sin(thetaref(k))*T ;
%          0 1 vref(k)*cos(thetaref(k))*T ;
%          0 0 1];
    
    % Construindo a matriz Abar:
%     Abar = A;
%     for i = 2 : N
%         Abar = [Abar ; A^i];
%     end
    
    % Construindo a matriz S:
    for col = 1 : N
        for lin = 1 : N
            if col <= lin
                if k+col > Kf
%                     A = [1 0 -vref(Kf)*sin(thetaref(Kf))*T ; 0 1 vref(Kf-1)*cos(thetaref(Kf))*T ; 0 0 1];
                    B = [cos(thetaref(Kf))*T 0 ; sin(thetaref(Kf))*T 0 ; 0 T];
                else
%                     A = [1 0 -vref(k+col)*sin(thetaref(k+col))*T ; 0 1 vref(k+col)*cos(thetaref(k+col))*T ; 0 0 1];
                    B = [cos(thetaref(k+col))*T 0 ; sin(thetaref(k+col))*T 0 ; 0 T];
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = A^(lin-col)*B;
            end
        end
    end
    
    % Construindo a matriz H:
    H = 2*(Rbar + Pbar + S'*Qbar*S);
    
    % Construindo o vetor xefrbar:
    qrefbar = [xref(k+1) ; yref(k+1) ; thetaref(k+1)];
    for i = 2 : N
        if k+i > Kf
            qrefbar = [qrefbar ; xref(Kf+1) ; yref(Kf+1) ; thetaref(Kf+1)];
        else
            qrefbar = [qrefbar ; xref(k+i) ; yref(k+i) ; thetaref(k+i)];        
        end
    end
    
    % Construindo o vetor urefbar:
    urefbar = [vref(k) ; wref(k)];
    for i = 1 : N-1
        if k+i > Kf
            urefbar = [urefbar ; vref(Kf) ; wref(Kf)];
        else
            urefbar = [urefbar ; vref(k+i) ; wref(k+i)];        
        end
    end
    
    % funcao f:
    f = 2*S'*Qbar*(Abar*q0 - qrefbar) - 2*Pbar*urefbar;
    
    u(:,k) = quadprog(H,f,[],[],[],[],-dbar,dbar,[],options);

    % u eh uma matriz de N X Tf*m elementos.
    % As primeiras duas linhas eh o controle do MPC.
    
    % Modelo cinematico:
    x(k+1) = x(k) + u(1,k)*cos(theta(k))*T;
    y(k+1) = y(k) + u(1,k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + u(2,k)*T;
  
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% FIM DA TRAJETORIA.

% disp('Erros finais dos estados:');
% disp('em x:');
% x(size(x,2))-xref(size(xref,2));
% disp('em y:');
% y(size(y,2))-yref(size(yref,2));
% disp('em theta:');
% theta(size(theta,2))-thetaref(size(thetaref,2));

% Graficos:

subplot(2,1,1);
hold on; box on; grid on;
plot(u(1,:),'r'); plot(vref,'-.k');
legend('v','v_{ref}',0);
title('Controle');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
plot(u(2,:),'r'); plot(wref,'-.k');
legend('w','w_{ref}',0);
hold off;

figure;
subplot(3,1,1);
hold on; box on; grid on;
plot(x,'r'); plot(xref,'-.k');
legend('x','x_{ref}',0);
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(y,'b'); plot(yref,'-.k');
legend('y','y_{ref}',0);
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(theta,'g'); plot(thetaref,'-.k');
legend('\theta','\theta_{ref}',0);
hold off;

figure;
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('traj.','referencia',0);
title('Trajetoria');
hold off;

figure;
hold on; box on; grid on;
plot(x-xref,'r');
plot(y-yref,'g');
plot(theta-thetaref,'b');
legend('e_x','e_y','e_{\theta}',0);
title('Erros');
hold off;
