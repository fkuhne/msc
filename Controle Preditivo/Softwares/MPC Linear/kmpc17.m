%
% kmpc17.m
%
% All constraints as L*u <= r.
% The robot does not reaches the end of the horizon,
% thus we can simplify the code.

% quadrado;
% oito;
% circulo;
path_u;
path;

clear all; clc;

% Load reference vectors:
load xref;
load yref;
load thetaref;
load vref;
load wref;

% TWIL data:
cr = 0.25; % distance between wheels.
wr = 0.075; % wheel radius.
mr = 1; % maximum rotation of motors in RPS.
mt = mr*2*pi*wr;% maximum tangent velocity of each wheel.
mv = (2*mt)/2; % maximum tangent velocity;
mw = (2*mt)/cr; % maximum angular velocity;

T = 0.1; % sampling period.

% System's dimensions:
n = 3; % state.
m = 2; % control.

N = 5; % prediction horizon.

% Weighting matrices:
Q = [1 0 0 ; 0 1 0 ; 0 0 0.5]; % state.
P = 30*Q; % terminal state cost.
R = 0.1*eye(m); % control.

% Control constraints:
vmax = mv; % v - superior limit.
vmin = -mv; % v - inferior limit.
wmax = mw; % w - superior limit.
wmin = -mw; % w - inferior limit.

% vmax = 1e5; % v - superior limit.
% vmin = -1e5; % v - inferior limit.
% wmax = 1e5; % w - superior limit.
% wmin = -1e5; % w - inferior limit.

% Building augmented matrices and vectors:
Qbar = Q;
Rbar = R;
L2 = [eye(m) ; -eye(m)]; L = L2;
d = [vmax ; wmax ; -vmin ; -wmin];
for i = 1 : N-1
    if (i==N) Qbar = blkdiag(Qbar,2^i*P);
    else Qbar = blkdiag(Qbar,2^i*Q);
    end
    Rbar = blkdiag(Rbar,R);
    L = blkdiag(L,L2);
    d = [d ; vmax ; wmax ; -vmin ; -wmin];
end
d2 = d;

% QUADPROG options:
options = optimset('LargeScale','off','Display','off');

% Final simulation time:
kf = size(vref,2);
dist = N; %distancia para o fim da trajetoria
kf2 = kf-dist; % tempo de simulacao
% kf2 = 999;
%dist = kf - kf2;

% Initial configuration of the WMR:
x(1) = -1;
y(1) = -1;
theta(1) = 0;

% initt = cputime;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% THIS IS THE BEGGINING OF THE TRAJECTORY LOOP:
for k = 1 : kf2
    
%     tic;
    
    % Building the augmented A matrix - Abar:
    A = mtx(thetaref(k),vref(k),T);
    Abar = A;
    for i = 1 : N-1 
        A = mtx(thetaref(k+i),vref(k+i),T)*A;
        Abar = [Abar ; A];
    end
    
    % Building S matrix:
    for col = 1 : N % column loop
        for lin = 1 : N % line loop
            if (lin > col) % if it's in the inferior part of S
                A = eye(n);
                for i = col : lin-1 A = mtx(thetaref(k+i),vref(k+i),T)*A; end
                if (k+col > kf) B = mtx(thetaref(kf),T);
                else B = mtx(thetaref(k+col-1),T);
                end
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = A*B;
            elseif (lin == col) % if it's in the main diagonal
                B = mtx(thetaref(k+col-1),T);
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = B;
            end
        end % end of lines
    end % end of columns
    
    % Building H matrix:
    H = 2*(S'*Qbar*S + Rbar);
    
    % Building urefbar vector:
    urefbar = [vref(k) ; wref(k) ; -vref(k) ; -wref(k)];
    for i = 1 : N-1 urefbar = [urefbar ; vref(k+i) ; wref(k+i) ; -vref(k+i) ; -wref(k+i)]; end
    
    % Constraint vector:
    d = d2 - urefbar;

    % Actual error:
    eq(:,k) = [x(k)     - xref(k);
               y(k)     - yref(k);
               theta(k) - thetaref(k)];
   
    % f function:
    f = 2*S'*Qbar*Abar*eq(:,k);

    % SOLVE THE OPTIMIZATION PROBLEMA THROUGH QUADRATIC PROGRAMMING:
    [eu,cost(k)] = quadprog(H,f,L,d,[],[],[],[],[],options); % com restricoes.
%     [eu,cost(k)] = quadprog(H,f,[],[],[],[],[],[],[],options); % SEM restricoes.

    % Independent term:   
    c = eq(:,k)'*Abar'*Qbar*Abar*eq(:,k);
    
    % Minimized cost function value:
    cost(k) = cost(k) + c;
    
    % Control applied to the plant:
    v(k) = eu(1) + vref(k);
    w(k) = eu(2) + wref(k);

    % Nonlinear kinematic model:
    x(k+1) = x(k) + v(k)*cos(theta(k))*T;
    y(k+1) = y(k) + v(k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + w(k)*T;
    
%     ktime(k) = toc;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% THIS IS THE END OF THE TRAJECTORY LOOP.

% disp('Tempo total de simulacao (em seg.):'); disp(cputime-initt);
% disp('Tempo medio de execucao de um passo (em seg.):'); disp(sum(ktime,2)/kf);
% 
% a = (x-xref(1:length(xref)-dist)).^2;
% b = (y-yref(1:length(yref)-dist)).^2;
% c = (theta-thetaref(1:length(thetaref)-dist)).^2;
% 
% epsilon = (sum(a,2)+sum(b,2)+sum(c,2))/kf2;
% disp('Erro medio quadratico:'); disp(epsilon);

disp('Final state errors:');
disp('in x:'); disp(x(length(x))-xref(length(xref)-dist));
disp('in y:'); disp(y(size(y,2))-yref(length(yref)-dist));
disp('in theta:'); disp(theta(length(theta))-thetaref(length(thetaref)-dist));

% Time vectors:
tu = (T:T:kf2*T)-T;
txr = [tu' ; tu(length(tu))+T];
tx = 0:T:kf2*T;

% Graficos:
figure('name','Funcao de custo Phi','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(tu,cost,'r');
ylabel('\Phi');
xlabel('tempo (s)');
hold off;

figure('name','Erros de estado','numbertitle','off');
hold on; box on; grid on;
plot(tx,x-xref(1:length(xref)-dist));
plot(tx,y-yref(1:length(xref)-dist),'--')
plot(tx,theta-thetaref(1:length(xref)-dist),'-.');
ylabel('Estados x, y e \theta (m)');
xlabel('tempo (s)');
hold off;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(tx,x,'r'); plot(txr,xref(1:length(xref)-dist),'-.k');
ylabel('x (m)');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(tx,y,'r'); plot(txr,yref(1:length(yref)-dist),'-.k');
ylabel('y (m)');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(tx,theta,'r'); plot(txr,thetaref(1:length(thetaref)-dist),'-.k');
ylabel('\theta (rad)');
xlabel('tempo (s)');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
stairs(tu,v,'r'); stairs(tu,vref(1:length(vref)-dist),'-.k');
ylabel('v (m/s)');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(tu,w,'r'); stairs(tu,wref(1:length(wref)-dist),'-.k');
ylabel('w (rad/s)');
xlabel('tempo (s)');
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; box on; grid on;
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x,y,'r');
plot(xref,yref,'-.k');
% plot(xref(1:401),yref(1:401),'-.k') %se a referencia e o circulo
xlabel('x (m)'); ylabel('y (m)');
hold off;
