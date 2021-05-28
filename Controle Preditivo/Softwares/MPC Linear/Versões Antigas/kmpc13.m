% kmpc13.m

oito; % gera a referencia.

clear all; close all; clc;

T = 0.1; % periodo de amostragem.

% Ordens do sistema:
n = 3; % do estado.
m = 2; % do controle.

a = 0.9; % agressividade da traj. de aproximacao.

N = 5; % horizonte de predicao.

% Matrizes de peso:
Q = [1 0 0 ; 0 1 0 ; 0 0 0.5]; % do estado.
R = eye(m); % do controle.

% Restricao no controle:
vmin = -1e4; % v - limite inferior.
vmax = 1e4; % v - limite superior.
wmin = -1e4; % w - limite inferior.
wmax = 1e4; % w - limite superior.

lmin = [vmin ; wmin];
lmax = [vmax ; wmax];

% Opcoes da funcao QUADPROG:
options = optimset('LargeScale', 'off', 'Display', 'off');

% Construindo as matrizs Qbar, Rbar e o vetor Lbar:
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

% Referencias:
load xref;
load yref;
load thetaref;
load vref;
load wref;

% Estado inicial:
x(1) = 0;
y(1) = -1;
theta(1) = pi/2;

v(1) = 0;
w(1) = 0;

% Instante final de simulacao:
kf = size(vref,2);

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% ESTE EH O LOOP DA TRAJETORIA:
for k = 1 : kf
    
    thetar(k) = theta(k);
    xr(k) = x(k);
    
    % Construindo a matriz Abar:
    if (k==1) 
        vr(k) = v(k);
    else 
        xr(k+1) = xref(k+1)-a*(x(k)-xref(k));
        vr(k) = (xr(k+1)-xr(k))/(cos(thetar(k))*T);
    end
    A = mtx(thetar(k),vr(k),T);
    Abar = A;
    for i = 1 : N-1
        if (k+i > kf)
            thetar(kf) = thetaref(kf) - a^i*(theta(k)-thetaref(k));
            xr(kf+1) = xref(kf+1) - a^(i+1)*(x(k)-xref(k));
            vr(kf) = (xr(kf+1)-xr(kf))/(cos(thetar(kf))*T);
            A = A*mtx(thetar(kf),vr(kf),T);
        else
            thetar(k+i) = thetaref(k+i) - a^i*(theta(k)-thetaref(k));
            xr(k+i+1) = xref(k+i+1) - a^(i+1)*(x(k)-xref(k));
            vr(k+i) = (xr(k+i+1)-xr(k+i))/(cos(thetar(k+i))*T);
            A = A*mtx(thetar(k+i),vr(k+i),T);
        end
        Abar = [Abar ; A];
    end
        
    % Construindo a matriz S:
    for col = 1 : N % loop das colunas
        for lin = 1 : N % loop das linhas
            if (lin > col) % se esta na triangular inferior
                A = eye(n);
                for i = col : lin-1
                    if (k+i > kf) 
                        thetar(kf) = thetaref(kf) - a^i*(theta(k)-thetaref(k));
                        xr(kf+1) = xref(kf+1) - a^(i+1)*(x(k)-xref(k));
                        vr(kf) = (xr(kf+1)-xr(kf))/(cos(thetar(kf))*T);
                        A = A*mtx(thetar(kf),vr(kf),T);
                    else
                        thetar(k+i) = thetaref(k+i) - a^i*(theta(k)-thetaref(k));
                        xr(k+i+1) = xref(k+i+1) - a^(i+1)*(x(k)-xref(k));
                        vr(k+i) = (xr(k+i+1)-xr(k+i))/(cos(thetar(k+i))*T);
                        A = A*mtx(thetar(k+i),vr(k+i),T);
                    end
                end
                if (k+col > kf) 
                    thetar(kf) = thetaref(kf) - a^i*(theta(k)-thetaref(k));
                    B = mtx(thetar(kf),T);
                else
                    thetar(k+col-1) = thetaref(k+col-1) - a^(col-1)*(theta(k)-thetaref(k));
                    B = mtx(thetar(k+col-1),T);
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = A*B;
            elseif (lin == col) % se esta na diagonal principal
                if (k+col > kf) 
                    thetar(kf) = thetaref(kf) - a^i*(theta(k)-thetaref(k));
                    B = mtx(thetar(kf),T);
                else 
                    thetar(k+col-1) = thetaref(k+col-1) - a^(col-1)*(theta(k)-thetaref(k));
                    B = mtx(thetar(k+col-1),T);
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = B;
            end
        end % fim das linhas
    end % fim das colunas
    
    % Construindo a matriz H:
    H = 2*(S'*Qbar*S + Rbar);
    
    % Construindo o vetor xrefbar:
    urefbar = [vref(k) ; wref(k)];
    for i = 1 : N-1
        if (k+i > kf)
            urefbar = [urefbar ; vref(kf) ; wref(kf)];
        else
            urefbar = [urefbar ; vref(k+i) ; wref(k+i)];        
        end
    end
    
    % Erro atual:
    eq(:,k) = [x(k) ; y(k) ; theta(k)] - [xref(k) ; yref(k) ; thetaref(k)];
   
    % funcao f:
    f = 2*S'*Qbar*Abar*eq(:,k);

    % vetores das restricoes no controle:
    dminbar = lminbar-urefbar; % limite inferior.
    dmaxbar = lmaxbar-urefbar; % limite superior.

    % RESOLVE O PROBLEMA DE OTIMIZACAO:
    eu = quadprog(H,f,[],[],[],[],dminbar,dmaxbar,[],options);
    
    % Controles na base original:
    v(k) = eu(1) + vref(k);
    w(k) = eu(2) + wref(k);
  
    % Modelo cinematico nao linear:
    x(k+1) = x(k) + v(k)*cos(theta(k))*T;
    y(k+1) = y(k) + v(k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + w(k)*T;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% FIM DA TRAJETORIA.

disp('Erros finais dos estados:');
disp('em x:'); disp(x(size(x,2))-xref(size(xref,2)));
disp('em y:'); disp(y(size(y,2))-yref(size(yref,2)));
disp('em theta:'); disp(theta(size(theta,2))-thetaref(size(thetaref,2)));

% Graficos:

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(x,'r'); plot(xref,'-.k');
legend('x','x_{ref}',1);
ylabel('x');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(y,'r'); plot(yref,'-.k');
legend('y','y_{ref}',1);
ylabel('y');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(theta,'r'); plot(thetaref,'-.k');
legend('\theta','\theta_{ref}',1);
ylabel('\theta');
xlabel('samples');
hold off;

figure('name','Erros em x, y e theta','numbertitle','off');
hold on; box on; grid on;
plot(eq(1,:),'k'); plot(eq(2,:),'-.k'); plot(eq(3,:),'--k');
legend('e_x','e_y','e_{\theta}',1);
ylabel('errors in x, y and \theta');
xlabel('samples');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
stairs(v,'r'); stairs(vref,'-.k');
legend('v','v_{ref}',1);
ylabel('v');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(w,'r'); stairs(wref,'-.k');
legend('w','w_{ref}',1);
ylabel('w');
xlabel('samples');
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('trajectory','reference',1);
xlabel('x'); ylabel('y');
hold off;
