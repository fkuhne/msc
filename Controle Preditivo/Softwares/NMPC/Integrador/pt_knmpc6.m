% pt_knmpc6.m
% posture stabilization.
% 
% Implementing the Julio's idea where we include
% an integrating term in the robot's dynamics.
%

close all; clear all; clc; warning off MATLAB:divideByZero

T = 0.1; % sampling period for control.

is = 5; % number of inter-sampling steps.
Ts = T/is; % sampling period for simulation.

N = 5; % prediction horizon.

p = 8; % augmented system ([X U]') dimension.

% FMINCON options:
options = optimset('LargeScale','off', 'Display','off');

% DEFINITIONS OF THE CONSTRAINTS:
% If no constraints were defined:
A = []; b = []; % A*z <= b.
Aeq = []; beq = []; % Aeq*z = beq.
lb = []; ub = []; % lb <= z <= ub.

% Amplitude constraint for states:
x_max = 1e5;
x_a_max = 1e5;
x_min = -1e5;
x_a_min = -1e5;
y_max = 1e5;
y_a_max = 1e5;
y_min = -1e5;        
y_a_min = -1e5;
theta_max = 1e5;
theta_a_max = 1e5;
theta_min = -1e5;
theta_a_min = -1e5;

% Amplitude constraint for controls:
delta_v_max = 1e5;
delta_v_min = -1e5;
delta_w_max = 1e5;
delta_w_min = -1e5;

% Defining amplitude constraints:
A_ampl = [eye(p) ; -eye(p)];
A = A_ampl;
b_ampl = [x_max ; x_a_max ; y_max ; y_a_max ; theta_max ; theta_a_max ; delta_v_max ; delta_w_max ; 
         -x_min ; -x_a_min ; -y_min ; -y_a_min ; -theta_min ; -theta_a_min ; -delta_v_min ; -delta_w_min]; 
b = b_ampl;
for i = 1 : N-1;
    A = blkdiag(A,A_ampl);
    b = [b ; b_ampl];
end

% Initial configuration of the WMR:
x(1) = -.2;
y(1) = 3;
theta(1) = 0;
x_a(1) = x(1);
y_a(1) = y(1);
theta_a(1) = theta(1);
delta_v(1) = 0;
delta_w(1) = 0;

% Final step:
kf = 49;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
   
    % Stacking the IC vector:
    if (k==1)
        z1 = [x(1) ; x_a(1) ; y(1) ; y_a(1) ; theta(1) ; theta_a(1) ; delta_v(1) ; delta_w(1)];
        z0 = z1;                               
        for i = 1 : N-1
            z0 = [z0 ; z1]; 
        end
    else 
        z0 = z;
    end
         
    [z,cost(k)] = fmincon('pt_cost',z0,A,b,Aeq,beq,lb,ub,'pt_nl_constr',options,N,z0);
   
    delta_v(k) = z(7);
    delta_w(k) = z(8);
    
    if (k==1)
        v(k) = delta_v(k);
        w(k) = delta_w(k);        
    else
        v(k) = v(k-1) + delta_v(k);
        w(k) = w(k-1) + delta_w(k);
    end
    
    for i = 1 : is
        ks = (k-1)*is+i; % inter-sampling step.
        x(ks+1) = x(ks) + v(k)*cos(theta(ks))*Ts + randn(1);
        y(ks+1) = y(ks) + v(k)*sin(theta(ks))*Ts + randn(1);
        theta(ks+1) = theta(ks) + w(k)*Ts + randn(1);        
    end
    
    z(1) = x(k*is+1);
%     z(2) = x_a(k*is+1);    
    z(3) = y(k*is+1);
%     z(4) = y_a(k*is+1);
    z(5) = theta(k*is+1);
%     z(6) = theta_a(k*is+1);
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

lx = length(x); % length of the state vectors.

disp('x final: '); disp(x(lx));
disp('y final: '); disp(y(lx));
disp('theta final: '); disp(theta(lx));

% Time vector:
tu = (T:T:kf*T)-T;
tx = 0:Ts:kf*T;

figure('name','Funcao de custo','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(tu,cost,'r');
ylabel('\Phi');
xlabel('tempo (s)');
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
xlabel('tempo (s)');
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
xlabel('tempo (s)');
hold off;

figure('name','Trajetoria no plano xy','numbertitle','off');
hold on; grid on; box on;
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x(lx),y(lx),'xk','markersize',11); plot(x(lx),y(lx),'.k','markersize',10);
plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13);
plot(x,y,'r');
xlabel('x (m)'); ylabel('y (m)');
hold off;
