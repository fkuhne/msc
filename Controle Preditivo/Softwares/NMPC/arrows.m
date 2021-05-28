% arrows.m
% place the arrows at initial and final configurations.
%

arrow([x(1) y(1)],[x(1)+0.1*cos(theta(1)) y(1)+0.1*sin(theta(1))],'length',10);
arrow([x(lx) y(lx)],[x(lx)+0.1*cos(theta(lx)) y(lx)+0.1*sin(theta(lx))],'length',10);
