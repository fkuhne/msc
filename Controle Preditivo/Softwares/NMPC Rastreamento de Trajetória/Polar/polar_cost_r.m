% Cost function for trajectory tracking
%
% TRAJECTORY TRACKING IN POLAR COORDINATES
%
% Phi = sum{ Xtil'*Q*Xtil - Util'*R*Util }

function Phi = polar_cost(z,N,k,kf,xref,yref,thetaref,vref,wref,z0);

p = 5; % Dimension of the augmented system.

% Polar cost function parameters:
l = 2;
h = 2;

R = 0.1*eye(2); % input weighting mtx.

Phi = 0;
for i = 0 : N-1
    
    if (k+i >= kf)
        e = sqrt((z(p*i+1)-xref(kf+1))^2 + (z(p*i+2)-yref(kf+1))^2);
        phi = atan2((z(p*i+2)-yref(kf+1)),(z(p*i+1)-xref(kf+1)));
        a = z(p*i+3) - phi - thetaref(kf+1);
        
        U = [z(p*i+4) - vref(kf); z(p*i+5) - wref(kf)];
    else
        e = sqrt((z(p*i+1)-xref(k+i+1))^2 + (z(p*i+2)-yref(k+i+1))^2);
        phi = atan2((z(p*i+2)-yref(k+i+1)),(z(p*i+1)-xref(k+i+1)));
        a = z(p*i+3) - phi - thetaref(k+i+1);
        
        U = [z(p*i+4) - vref(k+i); z(p*i+5) - wref(k+i)];
    end

    V = (1/2)*(l*e^2 + a^2 + h*phi^2) + U'*R*U;
    Phi = Phi + V;
end
