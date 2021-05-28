function qp = f(t,q)

% Control data (Lages, 1998):
g1 = 0.05;
g2 = 0.1;
h = 1.35;

eta1 = -g1*q(1)*cos(q(3));
eta2 = -g2*q(3) - g1*cos(q(3))*(sin(q(3))/q(3))*(q(3)-h*q(2));

ep = eta1*cos(q(3));
phip = eta1*(sin(q(3))/q(1));
alphap = eta1*(sin(q(3))/q(1)) + eta2;

qp = [ep ; phip ; alphap ; eta1 ; eta2];
