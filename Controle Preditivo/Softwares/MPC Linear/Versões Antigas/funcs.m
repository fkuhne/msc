function A = mtxA(thetaref,vref,T)

A = [1 0 -vref*sin(thetaref)*T ;
    0 1 vref*cos(thetaref)*T ;
    0 0 1 ];

function B = mtxB(thetaref,T)

B = [cos(thetaref)*T 0 ;
    sin(thetaref)*T 0 ;
    0               T];


function Z = mtxZ(thetaref)

Z = [cos(thetaref) sin(thetaref) 0 ;
    -sin(thetaref) cos(thetaref) 0 ;
    0              0             1];
