% pt_knmpc4.m
% posture stabilization.

close all; clear all; clc; warning off MATLAB:divideByZero

% TWIL data:
d = 0.25; % distance between wheels.
wr = 0.075; % wheel radius.
mr = 1; % maximum rotation of motors in RPS.
mt = mr*2*pi*wr;% maximum tangent velocity of each wheel.
mv = (2*mt)/2; % maximum linear velocity;
mw = (2*mt)/d; % maximum angular velocity;

T = 0.1; % sampling period for control.

is = 10; % number of inter-sampling steps.
Ts = T/is; % sampling period for simulation.

N = 5; % prediction horizon.

p = 5; % augmented system ([X U]') dimension.

% Final desired posture:
x_final = 0;
y_final = 0;
theta_final = 0;

% FMINCON options:
options = optimset('LargeScale','off', 'Display','off');

% DEFINITIONS OF THE CONSTRAINTS:
% If no constraints were defined:
A = []; b = []; % A*z <= b.
Aeq = []; beq = []; % Aeq*z = beq.
lb = []; ub = []; % lb <= z <= ub.

% Amplitude constraint for states:
x_max = 1e5;
x_min = -1e5;
y_max = 1e5;
y_min = -1e5;        
theta_max = 1e5;
theta_min = -1e5;

% Amplitude constraint for controls:
v_max = mv;
v_min = -mv;
w_max = mw;
w_min = -mw;

% Final step:
kf = 100;

% Initial configuration of the WMR:
x(1) = 0.000001;
y(1) = 3;
theta(1) = 0;
v(1) = 0;
w(1) = 0;

Q = [1 0 0 ; 0 1 0 ; 0 0 0.5]; % state weighting mtx
epsilon = 0;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
   
    % Stacking the IC vector:
    if (k==1)
        z1 = [x(1) ; y(1) ; theta(1) ; v(1) ; w(1)]; z0 = z1;                               
        for i = 1 : N-1
            z0 = [z0 ; z1]; 
        end
    else z0 = z; end
    
    epsilon = epsilon + [z0(1) z0(2) z0(3)]*Q*[z0(1);z0(2);z0(3)];

    % Defining amplitude constraints:
    A_ampl = [eye(p) ; -eye(p)]; A = A_ampl;
    b_ampl = [x_max ; y_max ; theta_max ; v_max ; w_max ; 
        -x_min ; -y_min ; -theta_min ; -v_min ; -w_min]; 
    b = b_ampl;
    for i = 1 : N-1;
        A = blkdiag(A,A_ampl);
        b = [b ; b_ampl];
    end
    A_ampl = A;
    b_ampl = b;
    
    [z,cost(k)] = fmincon('pt_cost',z0,A,b,Aeq,beq,lb,ub,'pt_nl_constr',options,N,z0,x_final,y_final,theta_final);
   
    v(k) = z(4);
    w(k) = z(5);
    
    for i = 1 : is
        ks = (k-1)*is+i; % inter-sampling step.
        x(ks+1) = x(ks) + v(k)*cos(theta(ks))*Ts;
        y(ks+1) = y(ks) + v(k)*sin(theta(ks))*Ts;
        theta(ks+1) = theta(ks) + w(k)*Ts;        
    end
    
    z(1) = x(k*is+1);
    z(2) = y(k*is+1);
    z(3) = theta(k*is+1);
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

lx = length(x); % length of the state vectors.

disp('x final: '); disp(x(lx));
disp('y final: '); disp(y(lx));
disp('theta final: '); disp(theta(lx));
disp('epsilon/kf'); disp(epsilon/kf);

% Time vector:
tu = (T:T:kf*T)-T;
tx = 0:Ts:kf*T;

figure('name','Funcao de custo','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(tu,cost,'r');
ylabel('\Phi');
xlabel('time (s)');
hold off;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(tx,x,'r');
ylabel('x (m)');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(tx,y,'r');
ylabel('y (m)');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(tx,theta,'r');
ylabel('\theta (m)');
xlabel('time (s)');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
stairs(tu,v,'r');
ylabel('v (m/s)');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(tu,w,'r');
ylabel('w (rad/s)');
xlabel('time (s)');
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; grid on; box on;
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13);
plot(x,y,'r');
plot(x(lx),y(lx),'xk','markersize',11); plot(x(lx),y(lx),'.k','markersize',10);
xlabel('x (m)'); ylabel('y (m)');
hold off;
