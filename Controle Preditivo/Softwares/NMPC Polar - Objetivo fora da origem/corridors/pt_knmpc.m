% pt_knmpc.m
%
% POSTURE STABILIZATION IN POLAR COORDINATES.
%
% two corridor.
%
clc; clear all; close all;

% Constants data:
T = 0.1; % sampling period for control.
p = 5; % augmented system ([X U]') dimension.

% TWIL data:
b = 0.25; % distance between wheels.
wr = 0.075; % wheel radius.
mr = 1; % maximum rotation of motors in RPS.
mt = mr*2*pi*wr;% maximum tangent velocity of each wheel.
mv = (2*mt)/2; % maximum linear velocity;
mw = (2*mt)/b; % maximum angular velocity;

% FMINCON options:
options = optimset('LargeScale','off', 'Display','off');

% Inter-sampling period for simulation:
is = 5; % number of inter-sampling steps.
Ts = T/is;

N = 5; % prediction horizon.

% DEFINITIONS OF THE CONSTRAINTS:
% If no constraints were defined:
A = []; b = []; % A*z <= b.
Aeq = []; beq = []; % Aeq*z = beq.
lb = []; ub = []; % lb <= z <= ub.

% Rate-of-change constraints for states:
delta_x = 1e5;
delta_y = 1e5;
delta_theta = 1e5;

% Rate-of-change constraints for controls:
delta_v = 1e5;
delta_w = 1e5;

% Amplitude constraint for states:
x_max = 1;
y_max = 5;
theta_max = 1e5;
theta_min = -1e5;

% Amplitude constraint for controls:
v_max = mv;
v_min = -mv;
w_max = mw;
w_min = -mw;

% Initial configuration of the WMR:
x(1) = -3;
y(1) = 4;
theta(1) = 0;
v(1) = 0;
w(1) = 0;

e(1) = sqrt(x(1)^2+y(1)^2);
phi(1) = atan2(y(1),x(1));
alpha(1) = theta(1) - phi(1);

% Final step:
kf = 29;

done = 0; % stop flag.
k = 1; % initial value of k.

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
while ~done
%for k = 1 : kf

    if (x < -0.5) % if the robot is still in the first half of the corridor
        x_min = -5;
        y_min = 3;        
    else % if arrives the middle of the corridor it can go down
        x_min = -1;
        y_min = -0.5;
    end
    
    % Defining amplitude constraints:
    A_ampl = [eye(p) ; -eye(p)]; A = A_ampl;
    b_ampl = [x_max ; y_max ; theta_max ; v_max ; w_max ; 
        -x_min ; -y_min ; -theta_min ; -v_min ; -w_min]; 
    b = b_ampl;
    for i = 1 : N-1;
        A = blkdiag(A,A_ampl);
        b = [b ; b_ampl];
    end
    
    % Stacking the IC vector:
    if (k==1)
        z1 = [e(1) ; phi(1) ; alpha(1) ; v(1) ; w(1)]; z0 = z1;                               
        for i = 1 : N-1
            z0 = [z0 ; z1]; 
        end
    else z0 = z; end
    
    [z,cost(k)] = fmincon('pt_cost',z0,A,b,Aeq,beq,lb,ub,'polar_nl_constr',options,N,z0);
   
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
    
    if ((abs(z(1)*cos(z(2))) < 1e-4) & (abs(z(1)*sin(z(2))) < 1e-4) & (abs(z(3)+z(2)) < 1e-4)) done=1; kf = k;
    else k = k + 1;
    end
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

x = e.*cos(phi);
y = e.*sin(phi);
theta = alpha + phi;

disp('x final: '); disp(x(size(x,2)));
disp('y final: '); disp(y(size(y,2)));
disp('theta final: '); disp(theta(size(theta,2)));

% Time vectors:
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
ylabel('\theta (rad)');
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
rectangle('Position',[-3,0,2,3],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
rectangle('Position',[-3,5,2,1],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
rectangle('Position',[-3,5,5,1],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
rectangle('Position',[1,0,1,5],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
rectangle('Position',[1,0,1,5],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
plot([-0.1,0.1],[0,0],'k'); % plot axes
plot([0,0],[-0.1,0.1],'k');
plot(x,y,'r');
plot(x(1),y(1),'.r','markersize',10);
xlabel('x (m)'); ylabel('y (m)');
axis([-4 3 -1 7]);
hold off;
