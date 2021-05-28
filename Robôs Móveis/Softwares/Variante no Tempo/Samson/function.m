function xp = f(t,x)

z1 = x(1)*cos(x(3)) + x(2)*sin(x(3));
z2 = -x(1)*sin(x(3)) + x(2)*cos(x(1));
z3 = x(3) + z2*sin(t);

w = -z3 - z2*cos(t);
v = -z1 + sin(t)*z3*w;

xp(1) = v*cos(x(3));
xp(2) = v*sin(x(3));
xp(3) = w;

xp = [xp(1) ; xp(2) ; xp(3) ; v ; w];
