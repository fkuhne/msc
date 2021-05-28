echo off;

T = 0.1;
r = 2;
v = 0.3;
w = 0;
Tf = 1000;

% Inicializacao:
xref(1) = 0;
yref(1) = 0;
thetaref(1) = 0;

for k = 1 : Tf
    
    if (k>310) w = v/r; end;
    if (k>360) w = 0; end
    if (k>640) w = -v/r; end
    if (k>690) w = 0; end    
    
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
% xlabel('x_{ref} (m)'); ylabel('y_{ref} (m)');
% % plot(0,0,'+w','markersize',20); plot(0,0,'+k','markersize',13);
% plot(xref,yref);
% axis equal;
% hold off;

% Salva trajetoria de referencia:
save xref.mat xref;
save yref.mat yref;
save thetaref.mat thetaref;

% Salva controle de referencia:
save vref.mat vref;
save wref.mat wref;
