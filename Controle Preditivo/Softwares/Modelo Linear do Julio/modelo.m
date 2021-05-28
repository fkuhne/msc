% Modelo do robo - Julio, Ortega, Camacho - 1999

echo off;
clear all; close all; clc;

V = 2; % velocidade linear
T = 0.1; % periodo amostral

% Inicializacao:
xnl(:,1) = 0;
ynl(:,1) = 0;
thetanl(:,1) = 0;

xl(:,1) = xnl(:,1);
yl(:,1) = ynl(:,1);
thetal(:,1) = thetanl(:,1);



for k = 1 : 100
    gamma(:,k) = k;
    if k > 33 gamma(:,k) = gamma(:,k)/3; end;
    if k > 66 gamma(:,k) = -gamma(:,k)/8; end;

% Modelo nao linear do sistema:
xnl(:,k+1) = xnl(:,k) + sin(V*T*gamma(:,k))/gamma(:,k);
ynl(:,k+1) = ynl(:,k) + (1-cos(V*T*gamma(:,k)))/gamma(:,k);
thetanl(:,k+1) = thetanl(:,k) + V*T*gamma(:,k);

% Modelo linear do sistema:
xl(:,k+1) = xl(:,k) + V*T;
yl(:,k+1) = yl(:,k) + ((V*T)*(V*T)/2) * gamma(:,k);
thetal(:,k+1) = thetal(:,k) + V*T * gamma(:,k);

end

hold on; box; grid;
plot(thetanl,'r');
plot(thetal,'.r');
plot(ynl,'b');
plot(yl,'.b');
legend('\theta Linear','\theta nao-Linear','y linear','y nao-linear',2);
hold off;

figure;
hold on; box; grid;
xlabel('x'); ylabel('y');
plot(xl,yl,'r');
plot(xnl,ynl);
legend('trajetoria linear','trajetoria nao-linear',2);
hold off;