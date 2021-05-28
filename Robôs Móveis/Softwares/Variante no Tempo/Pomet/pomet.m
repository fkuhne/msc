%
% Time-varying control law based on (Pomet et al., 1992)
%

clear all; close all; clc;

T = 0.01; % sampling period for control.

kf = 1499;

% Control data:
l = 4;
a = 2.1;

% Initial configuration of the WMR:
x(1) = 2;
y(1) = 2;
theta(1) = 0;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
    
    t = (k-1)*T;
   
    alfa = theta(k)/a;
    beta = -y(k)*(sin(t)-cos(t)) - (y(k)*sin(theta(k))+x(k)*cos(theta(k)))*cos(t)*sin(theta(k));
   
    v(k) = -x(k)*cos(theta(k)) - y(k)*sin(theta(k));
    w(k) = -a*cos(alfa)*sin(alfa) + a*l*(cos(alfa))^2*beta;
    
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