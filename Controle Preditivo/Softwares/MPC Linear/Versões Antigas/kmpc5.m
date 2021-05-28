%
% kmpc5.m
%
% Este programa realiza a otimizacao para o MPC
% atraves de programacao quadratica com uma
% linearizacao do modelo de erro a nivel cinematico.
% 
% A linearizacao e feita apenasem torno da referencia
% (xr,ur) a cada iteracao k.
%

% Gera as referencias (escolha uma):
% s;
% sinuosa;
% circulo;
oito;


clear all; close all; clc;

T = 0.1; % periodo de amostragem.

N = 3; % horizonte de predicao.

% Ordens do sistema:
n = 3; % do estado.
m = 2; % do controle.

% Matrizes de peso:
Q = eye(n); % do estado.
R = eye(m); % do controle.

% Restricao no controle:
vmin = 0.2; % v - limite inferior.
vmax = 0.4; % v - limite superior.
wmin = -0.4; % w - limite inferior.
wmax = 0.4; % w - limite superior.
% vmin = -inf; % v - limite inferior.
% vmax = inf; % v - limite superior.
% wmin = -inf; % w - limite inferior.
% wmax = inf; % w - limite superior.

lmin = [vmin ; wmin];
lmax = [vmax ; wmax];

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

% Construindo as matrizs Qbar, Rbar, Lbar e dbar:
Qbar = Q;
Rbar = R;
lminbar = lmin;
lmaxbar = lmax;
for i = 2 : N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    lminbar = [lminbar ; lmin];
    lmaxbar = [lmaxbar ; lmax];    
end

% Estados de referencia:
load xref;
load yref;
load thetaref;
qref(:,1) = [xref(1) ; yref(1) ; thetaref(1)];

% Controles de referencia:
load vref;
load wref;

% Inicializacao:
x(1) = -0.8;
y(1) = 1;
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
                    B = [T*cos(thetaref(Kf)) 0 ; T*sin(thetaref(Kf)) 0 ; 0 T];
                    A = [1 0 -vref(Kf)*sin(thetaref(Kf))*T ; 0 1 vref(Kf)*cos(thetaref(Kf))*T ; 0 0 1];
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
    
    % Construindo o vetor xefrbar:
    urefbar = [vref(k) ; wref(k)];
    for i = 1 : N-1
        if k+i > Kf
            urefbar = [urefbar ; vref(Kf) ; wref(Kf)];
        else
            urefbar = [urefbar ; vref(k+i) ; wref(k+i)];        
        end
    end
    
    % funcao f:
    f = 2*S'*Qbar*Abar*e0;
    
    [eu(:,k),fval(k)] = quadprog(H,f,[],[],[],[],lminbar-urefbar,lmaxbar-urefbar,[],options);
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

disp('Tempo total de simulacao (em seg.):');
endt = cputime-initt

disp('Erros finais dos estados:');
disp('em x:');
x(size(x,2))-xref(size(xref,2))
disp('em y:');
y(size(y,2))-yref(size(yref,2))
disp('em theta:');
theta(size(theta,2))-thetaref(size(thetaref,2))

% Graficos:

figure('name','Valor da funcao de custo Phi','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(fval,'k');
axis([0 840 -1.3 0.04]);
ylabel('\Phi_k');
xlabel('samples');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(v,'r'); plot(vref,'-.k');
legend('v','v_{ref}',0);
ylabel('v');
axis([0 840 0.18 0.42]);
hold off;
subplot(2,1,2);
hold on; box on; grid on;
plot(w,'r'); plot(wref,'-.k');
legend('w','w_{ref}',0);
ylabel('w');
xlabel('samples');
axis([0 840 -0.44 0.2]);
hold off;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(x,'r'); plot(xref,'-.k');
legend('x','x_{ref}',1);
ylabel('x');
axis([0 838 -5 5]);
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(y,'r'); plot(yref,'-.k');
legend('y','y_{ref}',1);
ylabel('y');
axis([0 838 -5 5]);
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(theta,'r'); plot(thetaref,'-.k');
legend('\theta','\theta_{ref}',1);
ylabel('\theta');
xlabel('samples');
axis([0 838 -0.1 2*pi+0.2]);
hold off;

figure('name','Erros em x, y e theta','numbertitle','off');
hold on; box on; grid on;
plot(ex,'k'); plot(ey,'-.k'); plot(etheta,'--k');
legend('e_x','e_y','e_{\theta}',1);
axis([0 838 -0.85 3.2]);
ylabel('errors in x, y and \theta');
xlabel('samples');
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('trajectory','reference',1);
xlabel('x'); ylabel('y');
axis([-2.5 2.5 -5 5]);
hold off;
