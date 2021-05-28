%
% kmpc16.m
%
% All constraints as L*u <= r.

traj_sinuosa; % Reference trajectory.

clear all; clc; %close all;

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

N = 10; % prediction horizon.
% Weighting matrices:
Q = [1 0 0; 0 1 0; 0 0 0.5]; % state.
R = 0.1*eye(m); % control.

% Building augmented matrices and vectors:
Qbar = Q;
Rbar = R;


for i = 2 : N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
end


% Final simulation time:
kf = size(vref,2);

% Initial configuration of the WMR:
x(1) = 0;
y(1) = -0.5;
theta(1) = pi/2;
vant=0;
want=0;

C=eye(n);
D=zeros(n,m);
C_hat=[C D];
for i=2 : N
    C_hat=blkdiag(C_hat,[C D]);
end

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% THIS IS THE BEGGINING OF THE TRAJECTORY LOOP:
for k = 1 : kf-N-1
    tic
    % Building the augmented A matrix - Abar:
    A = mtx(thetaref(k),vref(k),T);
    B = mtx(thetaref(k),T);
    A_hat = [A B;zeros(m,n) eye(m,m)];     %Retorna a matriz linearizada no ponto de operacao
    Pxx = A_hat;
    
    for i = 1 : N-1                                         %Cria a matriz Abarra(k)
        A_hat = A_hat*[mtx(thetaref(k+i),vref(k+i),T) mtx(thetaref(k+i),T); zeros(m,n) eye(m,m)];;
        Pxx = [Pxx ; A_hat];
    end
    
    % Building S matrix: (matriz Bbarra(k)
    for col = 1 : N % column loop
        for lin = 1 : N % line loop
            if (lin > col) % if it's in the inferior part of S
                A_hat = eye(n+m,n+m);
                for i = col : lin-1
                    A_hat = A_hat*[mtx(thetaref(k+i),vref(k+i),T) mtx(thetaref(k+i),T);zeros(m,n) eye(m,m)];;
                end
                B_hat = [mtx(thetaref(k+col-1),T);eye(m,m)];
                Hxx(lin*(n+m)-(n+m)+1:lin*(n+m), col*m-m+1:col*m) = A_hat*B_hat;
            elseif (lin == col) % if it's in the main diagonal
                B_hat = [mtx(thetaref(k+col-1),T);eye(m,m)];
                Hxx(lin*(n+m)-(n+m)+1:lin*(n+m), col*m-m+1:col*m) = B_hat;
            end
        end % end of lines
    end % end of columns
    
    r = [xref(k+1)-xref(k);
         yref(k+1)-yref(k);
         thetaref(k+1)-thetaref(k)];
     
    for i = 2 : N
        r = [r;
             xref(k+i)-xref(k+i-1);
             yref(k+i)-yref(k+i-1);
             thetaref(k+i)-thetaref(k+i-1)];
    end
   
    if k==1
        vref1=0;
        wref1=0;
    else
        vref1=vref(k-1);
        wref1=wref(k-1);
    end
    % Actual error:
    eq(:,k) = [x(k)     - xref(k);
               y(k)     - yref(k);
               theta(k) - thetaref(k);
               vant     - vref1;
               want     - wref1];
    
    H = C_hat*Hxx;
    P = C_hat*Pxx;
    
    K=inv(H'*Qbar*H+Rbar);
    delta_u=K*H'*(r-P*eq(:,k));
    
    % Control applied to the plant:
    v(k) = vant + delta_u(1);
    w(k) = want + delta_u(2);
    vant = v(k);
    want = w(k);
    
    % Modelo cinematico nao linear:
    x(k+1) = x(k) + v(k)*cos(theta(k))*T;
    y(k+1) = y(k) + v(k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + w(k)*T;
    z(k)=toc;
end

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
% THIS IS THE END OF THE TRAJECTORY LOOP.

disp('Final state errors:');
disp('in x:'); disp(x(size(x,2))-xref(size(xref,2)));
disp('in y:'); disp(y(size(y,2))-yref(size(yref,2)));
disp('in theta:'); disp(theta(size(theta,2))-thetaref(size(thetaref,2)));

% Time vectors:
tu = (T:T:(kf-N-1)*T)-T;
txr = [tu' ; tu(size(tu,2))+T];
tx = 0:T:(kf-N-1)*T;

% Graficos:
figure('name','States x, y and theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(tx,x,'r'); plot(tx,xref(1,1:kf-N),'-.k');
legend('x','x_{ref}',1);
ylabel('x');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(tx,y,'r'); plot(tx,yref(1,1:kf-N),'-.k');
legend('y','y_{ref}',1);
ylabel('y');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(tx,theta,'r'); plot(tx,thetaref(1,1:kf-N),'-.k');
legend('\theta','\theta_{ref}',1);
ylabel('\theta');
xlabel('time [s]');
hold off;

% figure('name','Erros em x, y e theta','numbertitle','off');
% hold on; box on; grid on;
% plot(eq(1,:),'k'); plot(eq(2,:),'-.k'); plot(eq(3,:),'--k');
% legend('e_x','e_y','e_{\theta}',1);
% ylabel('errors in x, y and \theta');
% xlabel('samples');
% hold off;

figure('name','Controls v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
stairs(tu,v,'r'); stairs(tu,vref(1,1:kf-N-1),'-.k');
legend('v','v_{ref}',1);
ylabel('v');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(tu,w,'r'); stairs(tu,wref(1,1:kf-N-1),'-.k');
legend('w','w_{ref}',1);
ylabel('w');
xlabel('time [s]');
hold off;

figure('name','XY Trajectory','numbertitle','off');
hold on; box on; grid on;
plot(x,y,'r');
plot(xref,yref,'-.k');
legend('trajectory','reference',1);
xlabel('x'); ylabel('y');
hold off;
