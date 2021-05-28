echo off;
clear all; close all; clc;

T = 0.1;
v = 0.3;
r = 2;
w = v/r;
Tf = 20*4*pi/v;

% Inicializacao:
xref(:,1) = 0;
yref(:,1) = 0;
thetaref(:,1) = 0;

xnl(:,1) = xref(:,1);
ynl(:,1) = yref(:,1);
thetanl(:,1) = thetaref(:,1);

xl(:,1) = xnl(:,1);
yl(:,1) = ynl(:,1);
thetal(:,1) = thetanl(:,1);

for k = 1 : Tf
   
    if (thetaref(:,k)>=2*pi) w = -w; end;
    
    % Modelo tradicional nao linear do robo:
    xref(:,k+1) = xref(:,k) + v * cos(thetaref(:,k)) * T;
    yref(:,k+1) = yref(:,k) + v * sin(thetaref(:,k)) * T;
    thetaref(:,k+1) = thetaref(:,k) + w * T;
 
%     % Geracao da entrada para o modelo do Julio:
%     dy(:,k) = y(:,k+1) - y(:,k);
%     gammay(:,k) = 2 * dy(:,k) / (v*T)^2;
% 
%     if (gammay(:,k)==0) gammay(:,k)=0.0001; end; % previne a divisao por zero no modelo do Julio
%     
%     dtheta(:,k) = theta(:,k+1) - theta(:,k);
%     gammatheta(:,k) = dtheta(:,k)/(v*T);
%     
%     % Modelo nao-linear do julio:
%     xnl(:,k+1) = xnl(:,k) + sin(v*T*gammay(:,k))/gammay(:,k);
%     ynl(:,k+1) = ynl(:,k) + (1-cos(v*T*gammay(:,k)))/gammay(:,k);
%     thetanl(:,k+1) = thetanl(:,k) + v*T*gammay(:,k);
    
end

gammaref = xref;

hold on; box on; grid on;
plot(xref,'g');
plot(yref,'b');
plot(thetaref,'r');
legend('x_{ref}','y_{ref}','\theta_{ref}',0);
xlabel('amostra'); ylabel('estados');
hold off;

figure;
hold on; box on; grid on;
xlabel('x_{ref}'); ylabel('y_{ref}');
plot(xref,yref);
hold off;

save yref.mat yref;
save thetaref.mat thetaref;