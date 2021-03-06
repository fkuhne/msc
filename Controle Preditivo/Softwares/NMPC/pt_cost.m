% Cost function for posture stabilization.
%
% Phi = sum{ X'*Q*X - U'*R*U }
%
% Input arg's:
%   z: vector of free variables
%   N: prediction horizon
%   z0: initial conditions

function Phi = pt_cost(z,N,z0,x_final,y_final,theta_final);

p = 5; % dimension of the augmented system ([X U]').
Q = [1 0 0 ; 0 1 0 ; 0 0 0.5]; % state weighting mtx.
P = 50*Q; % terminal state wighting mtx.
R = 0.1*eye(2); % input weighting mtx.

Phi = 0;
for i = 0 : N-1
    
    x = z(p*i+1) - x_final;
    y = z(p*i+2) - y_final;
    theta = z(p*i+3) - theta_final;
    X = [x ; y ; theta];
    
    e = sqrt(x^2+y^2);
    phi = atan2(y,x);
    alpha = theta - phi;
    X = [e ; phi ; alpha];
    
    U = [z(p*i+4) ; z(p*i+5)];
    
    Phi = Phi + X'*Q*X + U'*R*U; 
    
 %   if (i==N-1) Q = P; end
 %   Phi = Phi + X'*2^i*Q*X + U'*R*U;
end
