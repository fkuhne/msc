function B = mtxB(thetaref,T)

B = [cos(thetaref)*T 0 ;
     sin(thetaref)*T 0 ;
     0               T];
 