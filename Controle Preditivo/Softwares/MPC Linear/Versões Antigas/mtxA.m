function A = mtxA(thetaref,vref,T)

A = [1 0 -vref*sin(thetaref)*T ;
     0 1 vref*cos(thetaref)*T ;
     0 0 1 ];
 