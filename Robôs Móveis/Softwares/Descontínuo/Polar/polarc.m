%
% Discontinuous control law based on (Lages, 1998) in continuous time
%

clear all; close all; clc;

% Simulation time:
time = [0 300];

% Initial configuration of the WMR:
x0 = 0;
y0 = 4;
theta0 = 0;
v0 = 0;
w0 = 0;

e0 = sqrt(x0^2+y0^2);
phi0 = atan2(y0,x0);
alpha0 = theta0 - phi0;

IC = [e0 ; phi0 ; alpha0 ; v0 ; w0];

[t,q] = ode45('function',time,IC);

e = q(:,1);
phi = q(:,2);
alpha = q(:,3);
v = q(:,4);
w = q(:,5);

x = e.*cos(phi);
y = e.*sin(phi);
theta = alpha + phi;

figure('name','Estados x, y e theta','numbertitle','off');
subplot(3,1,1);
hold on; box on; grid on;
plot(t,x,'r');
ylabel('x');
hold off;
subplot(3,1,2);
hold on; box on; grid on;
plot(t,y,'r');
ylabel('y');
hold off;
subplot(3,1,3);
hold on; box on; grid on;
plot(t,theta,'r');
ylabel('\theta');
xlabel('time');
hold off;

figure('name','Controles v e w','numbertitle','off');
subplot(2,1,1);
hold on; box on; grid on;
plot(t,v,'r');
ylabel('v');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
plot(t,w,'r');
ylabel('w');
xlabel('time');
hold off;

figure('name','Trajetoria no plano XY','numbertitle','off');
plot(x,y,'r');
grid on; box on;
xlabel('x'); ylabel('y');
