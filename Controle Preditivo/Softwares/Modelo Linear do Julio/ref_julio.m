close all; clear all; clc

T = 0.1;
v = 30;

load gammaref;

yref(1) = 0;
thetaref(1) = 0;

for k = 1 : size(gammaref,2)
    
    % Modelo linear do julio:
    yref(k+1) = yref(k) + ((v*T)^2/2)*gammaref(k);
    thetaref(k+1) = thetaref(k) + v*T*gammaref(k);
    
end

save yref.mat yref;
save thetaref.mat thetaref;

% plot(yref);
% figure;
% plot(thetaref);
% figure;
% plot(gammaref);
