function xp = f(t,x)

% Control data (Sordalen, 1993):
l = 10;
a = 2.1;

alfa = x(3)/a;
beta = -x(2)*(sin(t)-cos(t)) - (x(2)*sin(x(3))+x(1)*cos(x(3)))*cos(t)*sin(x(3));

v = -x(1)*cos(x(3)) - x(2)*sin(x(3));
w = -a*cos(alfa)*sin(alfa) + a*l*(cos(alfa))^2*beta;

xp(1) = v*cos(x(3));
xp(2) = v*sin(x(3));
xp(3) = w;

xp = [xp(1) ; xp(2) ; xp(3) ; v ; w];
