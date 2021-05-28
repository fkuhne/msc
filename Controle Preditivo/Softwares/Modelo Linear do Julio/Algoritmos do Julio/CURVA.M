% rutina para calcular los ptos de referencia sobre
% las circunferencias de aproximacion. esta rutina 
% recibe un pto con orientacion y calcula el proximo
% en coor globales sobre la circunferencia

function [xs,ys,tetas]=curva(xe,ye,tetae,alfa,xc,yc,T,V)

% paso el pto xc,yc al sist de coordenadas en el pto xe ye con orien
% tacion tetae (xe ye esta en coor globales)

[xloc,yloc]=tr_coord(xc,yc,xe,ye,tetae);

% calculo la circunferencia

% determina el radio R de la circunferencia (el centro es siempre
% en coord locales (0,sign(yloc)*RR)

RR=(xloc^2 +yloc^2)/(2*yloc*sign(yloc));

% calcula el nuevo pto sobre la circunferencia

Dx=T*V;
Dy=(-sqrt(RR^2-(Dx)^2)+RR)*sign(yloc);

% calcula la nueva orientacion global computando la variacion del teta
% la nueva orientacion es tangente a la circ en el nuevo pto

fi=atan(Dx*sign(yloc)/sqrt(RR^2-Dx^2));
tetas=tetae+fi;

% transformo el pto calculado en coor globales

[xs,ys]=tr_coor2(Dx,Dy,xe,ye,tetae);
