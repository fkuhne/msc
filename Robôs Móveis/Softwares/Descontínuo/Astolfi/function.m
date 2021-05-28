function xp = f(t,x)

% Control data (Nijmeijer, 2001):
p1 = 1;
p2 = 25; 
p3 = -5;

v = -p1*x(1)/cos(x(3));
w = p2*x(2) + p3*x(3)/x(1);

xp(1) = v*cos(x(3));
xp(2) = v*sin(x(3));
xp(3) = w;

xp = [xp(1) ; xp(2) ; xp(3) ; v ; w];
