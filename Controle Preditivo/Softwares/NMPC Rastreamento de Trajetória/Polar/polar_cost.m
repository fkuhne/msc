% Cost function for trajectory tracking
%
% Phi = sum{ Xtil'*Q*Xtil + Util'*R*Util }

function Phi = polar_cost(z,N,k,kf,eref,phiref,alpharef,vref,wref,z0);

p = 5;

lambda = 2;
h = 2;

Q = [(1/2)*lambda 0 0 ; 0 (1/2)*h 0 ; 0 0 (1/2)]; % state weighting mtx.
R = 0.01*eye(2); % input weighting mtx.

Phi = 0;
for i = 0 : N-1
    
    if (k+i >= kf)
        e = z(p*i+1) - eref(kf+1);
        phi = z(p*i+2) - phiref(kf+1);
        alpha = z(p*i+3) - alpharef(kf+1);
        v = z(p*i+4) - vref(kf);
        w = z(p*i+5) - wref(kf);
        disp('k>kf');
    else
        e = z(p*i+1) - eref(k+i+1);
        phi = z(p*i+2) - phiref(k+i+1);
        alpha = z(p*i+3) - alpharef(k+i+1);
        v = z(p*i+4) - vref(k+i);
        w = z(p*i+5) - wref(k+i);
    end
    
    X = [e ; phi ; alpha];
    U = [v ; w];
    Phi = Phi + X'*Q*X + U'*R*U;    
end
