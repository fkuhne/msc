% tranforma ccordenadas globales en locales

function [xll,yll]=tr_coord(xg,yg,x,y,teta)

% calcula el angulo beta entre la recta que une (xg,yg) con (x,y) y
% el eje x del sistema

if xg > x
  beta= atan((yg-y)/(xg-x));
end
if xg < x
   beta=pi+atan((yg-y)/(xg-x));
end
if xg==x
  if yg > y
      beta=pi/2;
   else
      beta=-pi/2;
   end
end

% calcula xll e yll corden locales de xg,yg

alfa1=sqrt((yg-y)^2+(xg-x)^2);
xll=alfa1*cos(beta-teta);
yll=alfa1*sin(beta-teta);