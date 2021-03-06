%
% Discontinuous control law based on (Astolfi, 1996)
%

clear all; close all; clc;

T = 0.0001; % sampling period for control.

kf = 9999;

% Control data (Nijmeijer, 2001):
p1 = 1;
p2 = 25;
p3 = -5;

% Initial configuration of the WMR:
x(1) = 1;
y(1) = 2;
theta(1) = 0;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
    
    v(k) = -p1*x(k)/cos(theta(k));
    w(k) = p2*y(k) + p3*theta(k)/x(k);
    
    x(k+1) = x(k) + v(k)*cos(theta(k))*T;
    y(k+1) = y(k) + v(k)*sin(theta(k))*T;
    theta(k+1) = theta(k) + w(k)*T;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

% Time vectors:
tu = (T:T:kf*T)-T;
tx = 0:T:kf*T;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(tx,x,'r');
ylabel('x');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(tx,y,'r');
ylabel('y');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(tx,theta,'r');
ylabel('\theta');
xlabel('time');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
stairs(tu,v,'r');
ylabel('v');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(tu,w,'r');
ylabel('w');
xlabel('time');
hold off;

figure('name','Trajetoria no plano XY','numbertitle','off');
hold on; grid on; box on;
plot(x,y,'r');
xlabel('x'); ylabel('y');
hold off;

