echo off;
clear all; close all; clc;

T = 0.1;
v = 0.3;
w = 0;
Tf = 800;

% Inicializacao:
xref(1) = 0;
yref(1) = 0;
thetaref(1) = 0;

for k = 1 : Tf
    
     if (k>100) thetaref(k) = pi/2; end;
     if (k>200) thetaref(k) = pi; end;    
     if (k>300) thetaref(k) = -pi/2; end;    
     if (k>400) thetaref(k) = 0; end;         
     if (k>500) thetaref(k) = pi/2; end;    
     if (k>600) thetaref(k) = pi; end;    
     if (k>700) thetaref(k) = -pi/2; end;    
    
    vref(k) = v;
    wref(k) = w;
    
    % Modelo tradicional nao linear do robo:
    xref(k+1) = xref(k) + vref(k) * cos(thetaref(k)) * T;
    yref(k+1) = yref(k) + vref(k) * sin(thetaref(k)) * T;
    thetaref(k+1) = thetaref(k) + wref(k) * T;
  
end

% Reference state trajectories in polar coordinates:
eref = sqrt(xref.^2 + yref.^2);
phiref = atan2(yref,xref);
alpharef = thetaref-phiref;

% Save reference polar state trajectories:
save eref.mat eref;
save phiref.mat phiref;
save alpharef.mat alpharef;

% Salva trajetoria de referencia:
save xref.mat xref;
save yref.mat yref;
save thetaref.mat thetaref;

% Salva controle de referencia:
save vref.mat vref;
save wref.mat wref;
