% Nonlinear constraints
%
% Inequality constraints:
%   c <= []
% Equality constraints:
%   x(k+1) - x(k) - v(k)*cos(theta(k))*T = 0
%   y(k+1) - y(k) - v(k)*sin(theta(k))*T = 0
%   theta(k+1) - theta(k) - w(k)*T   = 0
%
% Input arg's:
%   z: vector of free variables
%   N: prediction horizon
%   z0: initial condition

function [c,ceq] = pt_nl_constr(z,N,z0,x_final,y_final,theta_final);

p = 5; % augmented system ([X U]') dimension.
T = 0.1;

c = []; % inequality constraint.

ceq = [z(1) - z0(1) - z(4)*cos(z0(3))*T ;
       z(2) - z0(2) - z(4)*sin(z0(3))*T ;
       z(3) - z0(3) -            z(5)*T];
for i = 1 : N-1
    nl = [z(p*i+1) - z(p*(i-1)+1) - z(p*i+4)*cos(z(p*(i-1)+3))*T ;
          z(p*i+2) - z(p*(i-1)+2) - z(p*i+4)*sin(z(p*(i-1)+3))*T ;
          z(p*i+3) - z(p*(i-1)+3) -                   z(p*i+5)*T];
    ceq = [ceq ; nl]; % equality constraint.
end
