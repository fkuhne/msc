% tranforma ccordenadas locales en globales

function [xg,yg]=tr_coor2(xll,yll,x,y,teta)


% calcula xg e yg corden globales de xll,yll
if abs(xll)> 0.0001
   beta=atan(yll/xll);
else
   beta=0;
end
dist=sqrt(xll^2+yll^2);
xg=dist*cos(teta+beta)+x;
yg=dist*sin(teta+beta)+y;