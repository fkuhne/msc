% Cost function for trajectory tracking
%
% Phi = sum{ Xtil'*Q*Xtil + Util'*R*Util }

function Phi = cost(z,N,k,kf,xref,yref,thetaref,vref,wref,z0);

Q = [1 0 0 ; 0 1 0 ; 0 0 0.5]; % state weighting mtx.
R = 0.1*eye(2); % input weighting mtx.

p = 5; % dimension of the augmented system ([x u]').

Phi = 0;
for i = 0 : N-1
    
    if (k+i >= kf)
        xtil = z(p*i+1) - xref(kf+1);
        ytil = z(p*i+2) - yref(kf+1);
        thetatil = z(p*i+3) - thetaref(kf+1);
        vtil = z(p*i+4) - vref(kf);
        wtil = z(p*i+5) - wref(kf);
    else
        xtil = z(p*i+1) - xref(k+i+1);
        ytil = z(p*i+2) - yref(k+i+1);
        thetatil = z(p*i+3) - thetaref(k+i+1);
        vtil = z(p*i+4) - vref(k+i);
        wtil = z(p*i+5) - wref(k+i);
    end

    X = [xtil ; ytil ; thetatil];
    
%     etil = sqrt(xtil^2+ytil^2);
%     phitil = atan2(ytil,xtil);
%     alphatil = thetatil - phitil;
%     X = [etil ; phitil ; alphatil];

     U = [vtil ; wtil];
    
     if (i==N-1) Q = 30*Q; end
     Phi = Phi + X'*2^i*Q*X + U'*R*U;
    
%     Phi = Phi + X'*Q*X + U'*R*U;    
end
