% pure pursuit strategy

function wpp = ppursuit(xpp,ypp,thetapp,xref,yref,vpp)

if (nargin ~= 6)
    error('PPURSUIT function must have 6 input arguments');
end

xc = cos(thetapp)*(xref-xpp) + sin(thetapp)*(yref-ypp);
yc = -sin(thetapp)*(xref-xpp) + cos(thetapp)*(yref-ypp);

r = (xc^2+yc^2)/(2*yc);

wpp = vpp/r;
