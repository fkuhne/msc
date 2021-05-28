%
% kmpc9.m
%
% Este programa realiza a otimizacao para o MPC
% atraves de programacao quadratica com uma
% linearizacao do modelo de erro a nivel cinematico.
% 
% A linearizacao e feita apenasem torno da referencia
% (xr,ur) a cada iteracao k.
%
% A linearizacao e variante no tempo tambem dentro
% do horizonte de predicao, ou seja, o modelo linearizado 
% (A(k+N),B(k+N)) eh diferente para cada N.
%
% Realiza a otimizacao sem restricoes e implementa o controle
% saturado na simulacao da planta.

% Gera as referencias (escolha uma):
% s;
oito;
% sinuosa;
% circulo;
% quadrado;
% reta;

clear all; close all; clc;

T = 0.1; % periodo de amostragem.

N = 5; % horizonte de predicao.

% Ordens do sistema:
n = 3; % do estado.
m = 2; % do controle.

% Matrizes de peso:
Q = [1 0 0 ; 0 1 0 ; 0 0 0.5]; % do estado.
R = 0.01*eye(m); % do controle.

% Restricao no controle:
vmin = -0.4; % v - limite inferior.
vmax = 0.4; % v - limite superior.
wmin = -0.4; % w - limite inferior.
wmax = 0.4; % w - limite superior.
% vmin = -inf; % v - limite inferior.
% vmax = inf; % v - limite superior.
% wmin = -inf; % w - limite inferior.
% wmax = inf; % w - limite superior.

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

% Construindo as matrizs Qbar, Rbar, Lbar e dbar:
Qbar = Q;
Rbar = R;
for i = 2 : N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
end

% Estados de referencia:
load xref;
load yref;
load thetaref;

% Controles de referencia:
load vref;
load wref;

% Estado inicial:
x(1) = 0;
y(1) = -1;
theta(1) = pi/2;

% Instante final de simulacao:
Kf = size(vref,2);

initt = cputime;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% ESTE EH O LOOP DA TRAJETORIA:
for k = 1 : Kf

    % Construindo a matriz Abar:
    A = mtx(thetaref(k),vref(k),T);
    Abar = A;
    for i = 1 : N-1
        if (k+i > Kf)
            A = A*mtx(thetaref(Kf),vref(Kf),T);
        else
            A = A*mtx(thetaref(k+i),vref(k+i),T);
        end
        Abar = [Abar ; A];
    end
    
    % Construindo a matriz S:
    for col = 1 : N % loop das colunas
        for lin = 1 : N % loop das linhas
            if (lin > col) % se esta na triangular inferior
                A = eye(n);
                for i = col : lin-1
                    if (k+i > Kf) A = A*mtx(thetaref(Kf),vref(Kf),T);
                    else A = A*mtx(thetaref(k+i),vref(k+i),T);
                    end
                end
                if (k+col > Kf) B = mtx(thetaref(Kf),T);
                else B = mtx(thetaref(k+col-1),T);
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = A*B;
            elseif (lin == col) % se esta na diagonal principal
                if (k+col > Kf) B = mtx(thetaref(Kf),T);
                else B = mtx(thetaref(k+col-1),T);
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = B;
            end
        end % fim das linhas
    end % fim das colunas
    
    % Construindo a matriz H:
    H = 2*(S'*Qbar*S+Rbar + Rbar);
    
    % Construindo o vetor xefrbar:
    urefbar = [vref(k) ; wref(k)];
    for i = 1 : N-1
        if (k+i > Kf)
            urefbar = [urefbar ; vref(Kf) ; wref(Kf)];
        else
            urefbar = [urefbar ; vref(k+i) ; wref(k+i)];        
        end
    end

    % Erro atual:
    eq(:,k) = [x(k) ; y(k) ; theta(k)] - [xref(k) ; yref(k) ; thetaref(k)];
    
    % funcao f:
    f = 2*S'*Qbar*Abar*eq(:,k);

    % RESOLVE O PROBLEMA DE OTIMIZACAO:
    [eu(:,k),fval(k)] = quadprog(H,f,[],[],[],[],[],[],[],options);

    % eu eh uma matriz de m*N X Kf elementos.
    % As primeiras duas linhas sao o controle do MPC.

    % termo independente:
    c = eq(:,k)'*Abar'*Qbar*Abar*eq(:,k);

    % Valor minimizado da funcao de custo:
    fval(k) = fval(k) + c;

    % Forca as saturacoes nos controles:
%     if (eu(1,k)>=vmax-vref(k)) eu(1,k) = vmax-vref(k); end
%     if (eu(1,k)<=vmin-vref(k)) eu(1,k) = vmin-vref(k); end
%     if (eu(2,k)>=wmax-wref(k)) eu(2,k) = wmax-wref(k); end
%     if (eu(2,k)<=wmin-wref(k)) eu(2,k) = wmin-wref(k); end
        
    % Controles na base original:
    v(k) = eu(1,k) + vref(k);
    w(k) = eu(2,k) + wref(k);
    
    % Modelo cinematico nao linear:
    x(k+1) = x(k) + v(k)*cos(theta(k))*T;
    y(k+1) = y(k) + v(k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + w(k)*T;

    % Modelo cinematico do erro:
%     ex(k+1) = ex(k) + eu(1,k)*cos(thetaref(k))*T;
%     ey(k+1) = ey(k) + eu(1,k)*sin(thetaref(k))*T;
%     etheta(k+1) = etheta(k) + eu(2,k)*T;

end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% FIM DA TRAJETORIA.

save xsat.mat x;
save ysat.mat y;
save thetasat.mat theta;
save vsat.mat v;
save wsat.mat w;
save fvalsat.mat fval;

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
% axis([0 840 -0.1 21]);
ylabel('\Phi(k)');
xlabel('samples');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(v,'r'); plot(vref,'-.k');
legend('v','v_{ref}',1);
ylabel('v');
% axis([0 840 0.18 0.42]);
hold off;
subplot(2,1,2);
hold on; box on; grid on;
plot(w,'r'); plot(wref,'-.k');
legend('w','w_{ref}',1);
ylabel('w');
xlabel('samples');
% axis([0 840 -0.44 0.2]);
hold off;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(x,'r'); plot(xref,'-.k');
legend('x','x_{ref}',1);
ylabel('x');
% axis([0 838 -5 5]);
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(y,'r'); plot(yref,'-.k');
legend('y','y_{ref}',1);
ylabel('y');
% axis([0 838 -5 5]);
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(theta,'r'); plot(thetaref,'-.k');
legend('\theta','\theta_{ref}',1);
ylabel('\theta');
xlabel('samples');
% axis([0 838 -0.1 2*pi+0.2]);
hold off;

figure('name','Erros em x, y e theta','numbertitle','off');
hold on; box on; grid on;
plot(eq(1,:),'k'); plot(eq(2,:),'-.k'); plot(eq(3,:),'--k');
legend('e_x','e_y','e_{\theta}',1);
% axis([0 838 -0.81 3*pi/2]);
ylabel('errors in x, y and \theta');
xlabel('samples');
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('trajectory','reference',1);
xlabel('x'); ylabel('y');
% axis([-2.5 2.5 -5 5]);
hold off;
