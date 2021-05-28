
close all; clear all; clc;

T = 0.1; % sampling period for control.
time = [0 T];

kf = 2499;

% Control data (Lages, 1998):
gamma1 = 0.05;
gamma2 = 0.1;
h = 1.35;

% Initial configuration of the WMR:
x(1) = -0.2;
y(1) = 3;
theta(1) = 0;

e(1) = sqrt(x(1)^2+y(1)^2);
phi(1) = atan2(y(1),x(1));
alpha(1) = theta(1) - phi(1);

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
   
    v(k) = -gamma1*e(k)*cos(alpha(k));
    w(k) = -gamma2*alpha(k)-gamma1*cos(alpha(k))*(sin(alpha(k))/alpha(k))*(alpha(k)-h*phi(k));
    
    e(k+1) = e(k) + v(k)*cos(alpha(k))*T;
    phi(k+1) = phi(k) + v(k)*(sin(alpha(k))/e(k))*T;
    alpha(k+1) = alpha(k) - v(k)*(sin(alpha(k))/e(k))*T + w(k)*T;
    
end
% -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE END OF THE TRAJECTORY.

x = e.*cos(phi);
y = e.*sin(phi);
theta = alpha + phi;

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
plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13); % the origin.
plot(x,y,'r');
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x(lx),y(lx),'xk','markersize',11); plot(x(lx),y(lx),'.k','markersize',10);
xlabel('x (m)'); ylabel('y (m)');
hold off;