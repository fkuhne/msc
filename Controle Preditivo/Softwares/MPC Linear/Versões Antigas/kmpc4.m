% Gera as referencias:
oito_ponto;
% quadrado;

clear all; close all; clc;

T = 0.1; % periodo de amostragem.

N = 3; % horizonte de predicao.

% Ordens do sistema:
n = 3; % do estado.
m = 2; % do controle.

% Matrizes de peso:
Q = eye(n); % do estado.
R = eye(m); % do controle.

% Matriz de peso da restricao:
L = eye(m);

% Restricao nos erros do controle:
vlim = inf;
wlim = inf;
ulim = [vlim ; wlim];

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

% Construindo as matrizs Qbar, Rbar, Lbar e dbar:
Qbar = Q;
Rbar = R;
Lbar = L;
dbar = ulim;
for i = 2 : N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    Lbar = blkdiag(Lbar,L);
    dbar = [dbar ; ulim];
end

Ltil = blkdiag(Lbar,-Lbar);

% Estados de referencia:
load xref;
load yref;
load thetaref;
qref(:,1) = [xref(1) ; yref(1) ; thetaref(1)];

% Controles de referencia:
load vref;
load wref;

% Inicializacao:
x(1) = 0;
y(1) = 2;
theta(1) = pi;
q(:,1) = [x(1) ; y(1) ; theta(1)];

% Matriz de transformacao:
Z = [cos(thetaref(1)) sin(thetaref(1)) 0 ;
    -sin(thetaref(1)) cos(thetaref(1)) 0 ;
    0 0 1];

% Erro inicial:
eq(:,1) = Z*(q(:,1)-qref(:,1));
 
% Instante final de simulacao:
Kf = size(vref,2);

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% ESTE EH O LOOP DA TRAJETORIA:
for k = 1 : Kf
    
    % Erro atual:
    e0 = eq(:,k);
    ex(k) = eq(1,k);
    ey(k) = eq(2,k);
    etheta(k) = eq(3,k);
    
    % Matriz de transformacao atual:
    Z = [cos(thetaref(k)) sin(thetaref(k)) 0 ;
        -sin(thetaref(k)) cos(thetaref(k)) 0 ;
        0 0 1];
    
    eq2(:,k) = inv(Z)*eq(:,k);
    
    % Estados atuais:
    x(k) = eq2(1,k) + xref(k);
    y(k) = eq2(2,k) + yref(k);
    theta(k) = eq2(3,k) + thetaref(k);
    
    % Matriz A(k) atual:
    A = [1 0 -vref(k)*sin(thetaref(k))*T ;
         0 1 vref(k)*cos(thetaref(k))*T ;
         0 0 1];
    
    % Construindo a matriz Abar:
    Abar = A;
    for i = 2 : N
        Abar = [Abar ; A^i];
    end
     
    % Construindo a matriz S:
    for col = 1 : N
        for lin = 1 : N
            if col <= lin
                if k+col > Kf
                    B = [T*cos(thetaref(Kf-1)) 0 ; T*sin(thetaref(Kf-1)) 0 ; 0 T];
                    A = [1 0 -vref(Kf)*sin(thetaref(Kf))*T ; 0 1 vref(Kf-1)*cos(thetaref(Kf))*T ; 0 0 1];
                 else
                    B = [T*cos(thetaref(k+col)) 0 ; T*sin(thetaref(k+col)) 0 ; 0 T];
                    A = [1 0 -vref(k+col)*sin(thetaref(k+col))*T ; 0 1 vref(k+col)*cos(thetaref(k+col))*T ; 0 0 1];
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
    
    % Modelo cinematico do erro:
    ex(k+1) = ex(k) + eu(1,k)*cos(thetaref(k))*T;
    ey(k+1) = ey(k) + eu(1,k)*sin(thetaref(k))*T;
    etheta(k+1) = etheta(k) + eu(2,k)*T;
    
    eq(:,k+1) = [ex(k+1) ; ey(k+1) ; etheta(k+1)];
    
    v(k) = eu(1,k) + vref(k);
    w(k) = eu(2,k) + wref(k);

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

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(v,'r'); plot(vref,'-.k');
legend('v','v_{ref}',0);
ylabel('v');
title('Controle');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
plot(w,'r'); plot(wref,'-.k');
legend('w','w_{ref}',0);
ylabel('w');
hold off;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(x,'r'); plot(xref,'-.k');
legend('x','x_{ref}',1);
ylabel('x');
% axis([0 840 -5 5]);
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(y,'r'); plot(yref,'-.k');
legend('y','y_{ref}',1);
ylabel('y');
% axis([0 840 -5 5]);
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(theta,'r'); plot(thetaref,'-.k');
legend('\theta','\theta_{ref}',1);
ylabel('\theta');
xlabel('samples');
% axis([0 840 -0.1 2*pi+0.2]);
hold off;

figure('name','Erros em x, y e theta','numbertitle','off');
hold on; box on; grid on;
plot(ex,'k'); plot(ey,'-.k'); plot(etheta,'--k');
legend('e_x','e_y','e_{\theta}',1);
% axis([0 840 -.75 1.5]);
hold off;
% title('Erros');

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('trajectory','reference',1);
xlabel('x'); ylabel('y');
axis([-2.5 2.5 -5 5]);
hold off;
% title('Trajetoria');
