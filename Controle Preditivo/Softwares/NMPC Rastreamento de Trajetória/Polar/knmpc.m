% knmpc.m
%
% TRAJECTORY TRACKING IN POLAR COORDINATES
%

polar_oito;
% polar_quadrado;
% polar_circulo;

clear all; close all; clc;

% Vector of the reference states:
load eref;
load phiref;
load alpharef;
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

% Systems dimension:
n = 3; % state.
m = 2; % control.
p = n+m; % augmented system ([X U]').

% FMINCON options:
options = optimset('LargeScale','off', 'Display','off');

is = 1; % number of inter-sampling steps.
Ts = T/is; % sampling period for simulation.

N = 5; % prediction horizon.

% Final step:
kf = length(vref);
dist = N; %distancia para o fim da trajetoria
kf2 = kf-dist; % tempo de simulacao

% DEFINITIONS OF THE CONSTRAINTS:
% If no constraints were defined, do it here:
A = []; b = []; % A*z <= b.
Aeq = []; beq = []; % Aeq*z = beq.
lb = []; ub = []; % lb <= z <= ub.

% Amplitude constraint for states:
x_max = 1e5;
y_max = 1e5;
theta_max = 1e5;
x_min = -1e5;
y_min = -1e5;
theta_min = -1e5;

% Amplitude constraint for controls:
v_max = mv;
w_max = mw;
v_min = -mv;
w_min = -mw;

% v_max = 1e5;
% v_min = -1e5;
% w_max = 1e5;
% w_min = -1e5;

A_ampl = [eye(p) ; -eye(p)]; A = A_ampl;
b_ampl = [x_max ; y_max ; theta_max ; v_max ; w_max ; 
          -x_min ; -y_min ; -theta_min ; -v_min ; -w_min]; 
b = b_ampl;
for i = 1 : N-1;
    A = blkdiag(A,A_ampl);
    b = [b ; b_ampl];
end

% Initial configuration of the WMR:
x(1) = 0;
y(1) = 0.5;
theta(1) = 0;
v(1) = 0;
w(1) = 0;

e(1) = sqrt(x(1)^2+y(1)^2);
phi(1) = atan2(y(1),x(1));
alpha(1) = theta(1) - phi(1);

initt = cputime;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf2

    tic;
    
    % Stacking the IC's vector:
    if (k==1)
        z1 = [e(1) ; phi(1) ; alpha(1) ; v(1) ; w(1)];
        z0 = z1;                               
        for i = 1 : N-1 z0 = [z0 ; z1]; end
    else z0 = z; end

    [z,cost(k)] = fmincon('polar_cost',z0,A,b,Aeq,beq,lb,ub,'polar_nl_constr',options,N,k,kf,eref,phiref,alpharef,vref,wref,z0);
   
    v(k) = z(4);
    w(k) = z(5);
    
    for i = 1 : is
        ks = (k-1)*is+i; % inter-sampling step.
        e(ks+1) = e(ks) + v(k)*cos(alpha(ks))*Ts;
        phi(ks+1) = phi(ks) + v(k)*(sin(alpha(ks))/e(ks))*Ts;
        alpha(ks+1) = alpha(ks) - v(k)*(sin(alpha(ks))/e(ks))*Ts + w(k)*Ts;        
    end
    
    z(1) = e(k*is+1);
    z(2) = phi(k*is+1);
    z(3) = alpha(k*is+1);
    
    ktime(k) = toc;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

meanktime = sum(ktime,2)/kf;
endt = cputime-initt;

disp('Tempo total de simulacao (em seg.):'); disp(endt);
disp('Tempo medio de execucao de um passo (em seg.):'); disp(meanktime);

% Load reference rectangular state trajectory:
load xref;
load yref;
load thetaref;

% Transform the actual trajectory from polar to rectangular:
x = e.*cos(phi);
y = e.*sin(phi);
theta = alpha + phi;

% Time vectors:
tu = (T:T:kf2*T)-T;
txr = [tu' ; tu(size(tu,2))+T];
tx = 0:Ts:kf2*T;

figure('name','Funcao de custo','numbertitle','off');
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
% legend('x','x_{ref}',1);
ylabel('x (m)');
% axis([0 kf 1.1*min(x) 1.1*max(x)]);
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(tx,y,'r'); plot(txr,yref(1:length(yref)-dist),'-.k');
% legend('y','y_{ref}',1);
ylabel('y (m)');
% axis([0 kf 1.1*min(y) 1.1*max(y)]);
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(tx,theta,'r'); plot(txr,thetaref(1:length(thetaref)-dist),'-.k');
% legend('\theta','\theta_{ref}',1);
ylabel('\theta (rad)');
xlabel('tempo (s)');
% axis([0 kf 1.1*min(theta) 1.1*max(theta)]);
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
stairs(tu,v,'r'); stairs(tu,vref(1:length(vref)-dist),'-.k');
ylabel('v (m/s)');
% axis([0 kf 0.29 0.41]);
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(tu,w,'r'); stairs(tu,wref(1:length(wref)-dist),'-.k');
ylabel('w (rad/s)');
xlabel('tempo (s)');
% axis([0 kf -0.44 0.2]);
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; box on; grid on;
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x,y,'r');
plot(xref,yref,'k');
% legend('trajectory','reference',1);
xlabel('x (m)'); ylabel('y (m)');
% axis([-2.5 2.5 -5 5]);
hold off;
