% Cost function for posture stabilization.
%
% POLAR COORDINATES
%
% Phi = Sum{ X'*Q*X + U'*R*U }
% based on Lyapunov function (1/2)*(lambda*e^2 + h*phi^2 + alpha^2).
%
% Input arg's:
%   z: vector of free variables
%   N: prediction horizon
%   z0: initial conditions

function Phi = pt_cost(z,N,z0);

p = 5; % dimension of the augmented system ([X U]').

lambda = 2;
h = 2;

Q = [(1/2)*lambda 0 0 ; 0 (1/2)*h 0 ; 0 0 (1/2)]; % state weighting mtx.
R = 0.1*eye(2); % input weighting mtx.

Phi = 0;
for i = 0 : N-1
    X = [z(p*i+1) ; z(p*i+2) ; z(p*i+3)];
    U = [z(p*i+4) ; z(p*i+5)];
%    if (i==N-1) Q = 50*Q; end
%    Phi = Phi + X'*2^(i+1)*Q*X + U'*R*U;
    Phi = Phi + X'*Q*X + U'*R*U;
end
