function Z = mtxZ(thetaref)

    Z = [cos(thetaref) sin(thetaref) 0 ;
        -sin(thetaref) cos(thetaref) 0 ;
        0              0             1];
     