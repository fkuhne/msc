% pt_knmpc3.m
% posture stabilization.
% crossing an "L" corridor.
% |----------------|
% |-------------   |
%              |   |
% |-------|    |   |
% |       |    |   |
% |       |    |   |
% |       |    |   |
% |-------|    |---|

close all; clear all; clc; warning off MATLAB:divideByZero;

% TWIL data:
b = 0.25; % distance between wheels.
wr = 0.075; % wheel radius.
mr = 1; % maximum rotation of motors in RPS.
mt = mr*2*pi*wr;% maximum tangent velocity of each wheel.
mv = (2*mt)/2; % maximum linear velocity;
mw = (2*mt)/b; % maximum angular velocity;

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

% Final step:
kf = 249;

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

% v_max = 1e5;
% v_min = -1e5;
% w_max = 1e5;
% w_min = -1e5;

% Initial configuration of the WMR:
x(1) = -4;
y(1) = 3.5;
theta(1) = pi;
v(1) = 0;
w(1) = 0;

done = 0; % stop flag.
k = 1; % initial value of k.

initt = cputime;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
% while ~done
for k = 1 : kf
   
    tic;
    
    % Stacking the IC vector:
    if (k==1)
        z1 = [x(1) ; y(1) ; theta(1) ; v(1) ; w(1)]; z0 = z1;                               
        for i = 1 : N-1
            z0 = [z0 ; z1]; 
        end
    else z0 = z; end

    if (z0(1) < -1) % if the robot is still in the first half of the corridor
%         x_max = 1;
%         x_min = -1e5;
%         y_max = 5;
%         y_min = 3;
        y_final = 3;
    else % if arrives the middle of the corridor it can go down
%         x_max = 1;
%         x_min = -1;
%         y_max = 5;
%         y_min = -1e5;
        y_final = 0;
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
    A_ampl = A;
    b_ampl = b;
    
    % Defining rate-of-change constraints:
%     A_rate = [eye(p) ; -eye(p)]; A = A_rate;
%     b_rate = [delta_x ; delta_y ; delta_theta ; delta_v ; delta_w ;
%               delta_x ; delta_y ; delta_theta ; delta_v ; delta_w];
%     b = b_rate + [z0(1) ; z0(2) ; z0(3) ; z0(4) ; z0(5) ;
%                   -z0(1) ; -z0(2) ; -z0(3) ; -z0(4) ; -z0(5)];
%     for i = 1 : N-1
%         z_rate = [z0(p*i+1) ; z0(p*i+2) ; z0(p*i+3) ; z0(p*i+4) ; z0(p*i+5) ;
%                   -z0(p*i+1) ; -z0(p*i+2) ; -z0(p*i+3) ; -z0(p*i+4) ; -z0(p*i+5)];
%         A = blkdiag(A,A_rate);
%         b = [b ; b_rate+z_rate];
%     end
%     A_rate = A;
%     b_rate = b;
%     
%     Defining actual constraints:
%     A = [A_ampl ; A_rate];
%     b = [b_ampl ; b_rate];
         
    [z,cost(k),exitflag,output,lambda] = fmincon('pt_cost',z0,A,b,Aeq,beq,lb,ub,'pt_nl_constr',options,N,z0,x_final,y_final,theta_final);
   
    if (~exitflag) disp('the objective function has not converged!'); end;
    
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
    
%     if (((abs(z(1))) < 1e-3) & (abs(z(2)) < 1e-3) & (abs(z(3)) < 1e-3)) done = 1; kf = k;
%     else k = k + 1;
%     end

    ktime(k) = toc;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

lx = length(x); % length of the state vectors.

disp('tempo medio em cada k: '); disp(sum(ktime,2)/kf);
disp('tempo total da trajetoria: '); disp(cputime-initt);

disp('x final: '); disp(x(lx));
disp('y final: '); disp(y(lx));
disp('theta final: '); disp(theta(lx));

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
rectangle('Position',[-4,0,3,3],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
rectangle('Position',[-4,5,6,1],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
rectangle('Position',[1,0,1,6],'FaceColor',[0.753 0.753 0.753],'LineStyle','none');
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x(lx),y(lx),'xk','markersize',11); plot(x(lx),y(lx),'.k','markersize',10);
plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13);
plot(x,y,'r');
xlabel('x (m)'); ylabel('y (m)');
axis([-4.5 2.5 -0.5 6.5]);
hold off;
