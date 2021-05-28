% Cost function for posture stabilization.
%
% Phi = sum{ X'*Q*X - U'*R*U }
%
% Implementing the Julio's idea where we include
% an integrating term in the robot's dynamics.
%

function Phi = pt_cost(z,N,z0);

p = 8; % dimension of the augmented system ([X U]').
Q = [1 0 0 0 0 0 ;
     0 .1 0 0 0 0 ;
     0 0 1 0 0 0 ;
     0 0 0 .1 0 0 ;
     0 0 0 0 .5 0 ;
     0 0 0 0 0 .05]; % state weighting mtx.
R = .1*eye(2); % input weighting mtx.

Phi = 0;
for i = 0 : N-1
    
    X = [z(p*i+1) ; % x 
         z(p*i+2) ; % x_a
         z(p*i+3) ; % y
         z(p*i+4) ; % y_a
         z(p*i+5) ; % theta
         z(p*i+6)]; % theta_a
    
    U = [z(p*i+7) ; % delta_v
         z(p*i+8)]; % delta_w
    
    % Cost function:
    Phi = Phi + X'*Q*X + U'*R*U; 
    
end
