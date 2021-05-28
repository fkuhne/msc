% POLAR OITO.

echo off;
clear all; close all; clc;

T = 0.1;
v = 0.3; 
r = 0.5; 
w = v/r; 
Tf = 4*pi/(T*w);

% Initialization:
xref(1) = 0;
yref(1) = 0;
thetaref(1) = 0;

for k = 1 : Tf
    
    if (thetaref(k)>=2*pi) w = -w; end;
    
    vref(k) = v;
    wref(k) = w;
    
    % Nonlinear kinematic model:
    xref(k+1) = xref(k) + vref(k)*cos(thetaref(k))*T;
    yref(k+1) = yref(k) + vref(k)*sin(thetaref(k))*T;
    thetaref(k+1) = thetaref(k) + wref(k)*T;
  
end

% Reference state trajectories in polar coordinates:
eref = sqrt(xref.^2 + yref.^2);
phiref = atan2(yref,xref);
alpharef = thetaref-phiref;

% Save reference polar state trajectories:
save eref.mat eref;
save phiref.mat phiref;
save alpharef.mat alpharef;

% Save reference rectangular state trajectories:
save xref.mat xref;
save yref.mat yref;
save thetaref.mat thetaref;

% Save reference control:
save vref.mat vref;
save wref.mat wref;
