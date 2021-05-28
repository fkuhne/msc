echo off;

T = 0.1;

r = 2; % raio da trajetoria
dfc = 2*pi*r; % circunferencia da trajetoria
tfc = 40; % tempo para percorrer a circunferencia em segundos
v = dfc/tfc; % velocidade para percorrer a circunferencia
%v = 0.3;
w = v/r; % velocidade angular
voltas = 4; % numero de voltas
Tf = tfc/T*voltas;

% Inicializacao:
xref(1) = 0;
yref(1) = -2;
thetaref(1) = 0;

for k = 1 : Tf
    
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
% figure;
% hold on; box on; grid on;
% xlabel('x_{ref}'); ylabel('y_{ref}');
% plot(xref,yref);
% hold off;

% Salva trajetoria de referencia:
save xref.mat xref;
save yref.mat yref;
save thetaref.mat thetaref;

% Salva controle de referencia:
save vref.mat vref;
save wref.mat wref;
