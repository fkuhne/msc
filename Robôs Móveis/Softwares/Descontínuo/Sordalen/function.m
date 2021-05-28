function zp = f(t,z)

tol = 1e-6;
maxinf = 1e3;

x = z(1);
y = z(2);
theta = z(3);

% Control data (Sordalen, TAC'1992):
lambda = 1.35;
k = 4.6;

beta = y/x;

if ((x==0) && (y==0))
    thetad = 0;
else
    thetad = 2*atan2(y,x);
end

if (abs(y)<tol)
    a = x;
else
    r = (x^2+y^2)/(2*y);
    a = r*thetad;
end

e = theta - thetad;

n = floor((e+pi)/(2*pi));

alpha = e - 2*pi*n;

if (abs(beta)<tol) b_1 = cos(alpha) + (pi/2)*abs(sin(alpha));
elseif (abs(beta)>maxinf) b_1 = cos(alpha) - (pi/2)*abs(sin(alpha));    
else b_1 = cos(theta)*(thetad/beta-1) + sin(theta)*((thetad/2)*(1-1/beta^2)+1/beta);
end

if (abs(beta)<tol) b_2 = -(2/x)*sin(theta);
elseif (abs(beta)>maxinf) b_2 = 0;
else b_2 = cos(theta)*((2*beta)/(x*(1+beta^2))) - sin(theta)*(2/(x*(1+beta^2)));
end

% Control law:
v = -lambda*b_1*a;
w = -b_2*v - k*alpha;

% Kinematic model of the WMR:
xp = v*cos(theta);
yp = v*sin(theta);
thetap = w;

% Return a vector with states and controls over all the simulation time:
zp = [xp ; yp ; thetap ; v ; w];
