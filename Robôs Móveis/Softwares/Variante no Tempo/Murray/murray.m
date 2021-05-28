%
% Time-varying control law based on (Murray et al., 1997)
%

clear all; close all; clc;

T = 0.005; % sampling period for control.

kf = 5999;

% Initial configuration of the WMR:
x(1) = 0;
y(1) = -3.2;
theta(1) = pi/4;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
   
    t = (k-1)*T;
    
    x_1 = theta(k);
    x_2 = x(k)*cos(theta(k)) + y(k)*sin(theta(k));
    x_3 = x(k)*sin(theta(k)) - y(k)*cos(theta(k));
    
    u_1 = -x_1 + x_3*cos(t);
    u_2 = -x_2 + x_3^2*sin(t);

    w(k) = u_1;
    v(k) = u_2 + w(k)*x_3;
    
    x(k+1) = x(k) + v(k)*cos(theta(k))*T;
    y(k+1) = y(k) + v(k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + w(k)*T;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

lx = length(x); % length of the state vectors.

disp('x final: '); disp(x(lx));
disp('y final: '); disp(y(lx));
disp('theta final: '); disp(theta(lx));

% Time vectors:
tu = (T:T:kf*T)-T;
tx = 0:T:kf*T;

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

figure('name','Trajetoria no plano XY','numbertitle','off');
hold on; grid on; box on;
plot(x,y,'r');
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x(lx),y(lx),'xk','markersize',11); plot(x(lx),y(lx),'.k','markersize',10);
plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13);
xlabel('x (m)'); ylabel('y (m)');
hold off;
