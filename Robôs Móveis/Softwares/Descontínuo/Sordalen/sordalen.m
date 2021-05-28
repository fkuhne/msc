clear all; clc;

tol = 1e-6;
maxinf = 1e3;

T = 0.001; % sampling period for control.
kf = 9999; % number of steps.

% Control data (Sordalen, 1991):
lambda = 1.35;
h = 4.6;

% Initial configuration of the WMR:
x(1) = -1;
y(1) = 3;
theta(1) = 0;

% -.-.-.-.-.-.-.-.-.-.-.-.-.-.
% THIS IS THE TRAJECTORY LOOP:
for k = 1 : kf
    
    beta = y(k)/x(k);
        
    if ((x(k)==0) & (y(k)==0)) thetad = 0;
    else thetad = 2*atan2(y(k),x(k));
    end
    
    if (y(k)==0) a = x(k);
    else
        r = (x(k)^2+y(k)^2)/(2*y(k));
        a = r*thetad;
    end
    
    e = theta(k) - thetad;
    n = floor((e+pi)/(2*pi));
    alpha = e - 2*pi*n;
    
    if (abs(beta)<tol) b_1 = cos(alpha) + (pi/2)*abs(sin(alpha));
    elseif (abs(beta)>maxinf) b_1 = cos(alpha) - (pi/2)*abs(sin(alpha));    
    else b_1 = cos(theta(k))*(thetad/beta-1) + sin(theta(k))*((thetad/2)*(1-1/beta^2)+1/beta);
    end
    
    if (abs(beta)<tol) b_2 = -(2/x)*sin(theta);
    elseif (abs(beta)>maxinf) b_2 = 0;
    else b_2 = cos(theta(k))*((2*beta)/(x(k)*(1+beta^2))) - sin(theta(k))*(2/(x(k)*(1+beta^2)));
    end
     
    v(k) = -lambda*b_1*a;
    w(k) = -b_2*v(k) - h*alpha;
    
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
plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13); % plot the origin.
plot(x,y,'r');
plot(x(1),y(1),'ok','markersize',10); plot(x(1),y(1),'.k','markersize',10);
plot(x(lx),y(lx),'xk','markersize',11); plot(x(lx),y(lx),'.k','markersize',10);
xlabel('x (m)'); ylabel('y (m)');
hold off;
