% Nonlinear constraints
%
% Implementing the Julio's idea where we include
% an integrating term in the robot's dynamics.
%

function [c,ceq] = pt_nl_constr(z,N,z0);

p = 8; % augmented system ([X U]') dimension.
T = 0.1;

c = []; % inequality constraint.

ceq = [z(1) - 2*z0(1) + z0(2) - z(7)*cos(z0(5))*T ; % x
       z(2) - z0(1) ;                               % x_a
       z(3) - 2*z0(3) + z0(4) - z(7)*sin(z0(5))*T ; % y
       z(4) - z0(3) ;                               % y_a
       z(5) - 2*z0(5) + z0(6) - z(8)*T ;            % theta         
       z(6) - z0(5)];                               % theta_a

for i = 1 : N-1
    nl = [z(p*i+1) - 2*z(p*(i-1)+1) + z(p*(i-1)+2) - z(p*i+7)*cos(z(p*(i-1)+5))*T ; % x
          z(p*i+2) - z(p*(i-1)+1) ;                                                 % x_a
          z(p*i+3) - 2*z(p*(i-1)+3) + z(p*(i-1)+4) - z(p*i+7)*sin(z(p*(i-1)+5))*T ; % y
          z(p*i+4) - z(p*(i-1)+3) ;                                                 % y_a
          z(p*i+5) - 2*z(p*(i-1)+5) + z(p*(i-1)+6) - z(p*i+8)*T ;                   % theta
          z(p*i+6) - z(p*(i-1)+5)];                                                 % theta_a
          
    ceq = [ceq ; nl]; % equality constraint.
end
