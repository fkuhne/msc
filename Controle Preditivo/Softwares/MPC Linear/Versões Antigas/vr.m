close all; clear all; clc;

load xref;
load yref;
load thetaref;
load vref;
load wref;

T = 0.1;

for k = 1 : size(xref,2)-1
    vrefx(k) = (xref(k+1)-xref(k))/(cos(thetaref(k))*T);
end

for k = 1 : size(xref,2)-1
    wref2(k) = (thetaref(k+1)-thetaref(k))/T;
end

hold; grid; box;
plot(wref,'-.k');
plot(wref2,'.r');
