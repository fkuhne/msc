%
% Time-varying control law based on (Pomet et al., 1992) in continuous time
%

clear all; close all; clc;

% Simulation time:
time = [0 100];

% Initial configuration of the WMR:
u0 = [0 ; 0];
x0 = [0 ; 2 ; 0];
CI = [x0 ; u0];

[t,r] = ode45('function',time,CI);

x = r(:,1);
y = r(:,2);
theta = r(:,3);
v = r(:,4);
w = r(:,5);

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
stairs(t,v,'r');
ylabel('v');
hold off;
subplot(2,1,2);
hold on; box on; grid on;
stairs(t,w,'r');
ylabel('w');
xlabel('time');
hold off;

figure('name','Trajetoria no plano XY','numbertitle','off');
plot(x,y,'r');
grid on; box on;
xlabel('x'); ylabel('y');


