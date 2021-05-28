echo off;
clear all; close all; clc;

T = 0.1;
v = 1; 
r = 2; 
w = v/r; 
Tf = 1000;

% Inicializacao:
xref(1) = 0;
yref(1) = 0;
thetaref(1) = 0;

for k = 1 : Tf
    
    if (yref(k)>=3) w = -v/r; end
%     if ((thetaref(k)>=0) && (xref(k)>3)) w = 0; end
    if ((xref(k)>5)) w = v/(2*r); end
    
    vref(k) = v;
    wref(k) = w;
    
    % Modelo tradicional nao linear do robo:
    xref(k+1) = xref(k) + vref(k) * cos(thetaref(k)) * T;
    yref(k+1) = yref(k) + vref(k) * sin(thetaref(k)) * T;
    thetaref(k+1) = thetaref(k) + wref(k) * T;
  
end

% Graficos:
% hold on; box on; grid on;
% plot(xref,'g');
% plot(yref,'b');
% plot(thetaref,'r');
% legend('x_{ref}','y_{ref}','\theta_{ref}',0);
% xlabel('amostra'); ylabel('estados');
% hold off;
% 
figure;
hold on; box on; grid on;
xlabel('x_{ref}'); ylabel('y_{ref}');
plot(xref,yref);
hold off;

% Salva trajetoria de referencia:
save xref.mat xref;
save yref.mat yref;
save thetaref.mat thetaref;

% Salva controle de referencia:
save vref.mat vref;
save wref.mat wref;
