%
% Discontinuous control law based on (Sordalen, 1991) in continuous time
%

clear all; close all; clc;

% Simulation time:
time = [0 10];

% Initial posture of the WMR:
x0 = 0.5;
y0 = 1;
theta0 = pi/2;

% Initial control of the WMR:
v0 = 0;
w0 = 0;

% Initial conditions:
IC = [x0 ; y0 ; theta0 ; v0 ; w0];

% SOLVE THE SISTEM:
[t,q] = ode45('function',time,IC);

% ...and the result is:
x = q(:,1);
y = q(:,2);
theta = q(:,3);
v = q(:,4);
w = q(:,5);

% Graphical results:
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


