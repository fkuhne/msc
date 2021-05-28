% Este programa simula un robor movil de cuatro ruedas tipo LABMATE controlado por un
% GPC multivariable basado en el modelo local lineal con variables medidas teta e y
% y variable de control la curvatura gama
% El modelo sigue una referencia creada a partir de arcos de circunferencia tipo 
% pure pursuit

% defino el numero de puntos de simulacion, la condicion inicial
% x0, y0, teta0 y el look ahead (alfa)
clear all

%oito; % gera a referencia.
%circulo;
%espiral;
quadrado;
%reta;
%s;
%sinuosa;

xx=xref;
yy=yref;

puntos=390;%size(xx,2);
alfa=0.5;
x0=0;
y0=0;
teta0=0.01;
%teta0=thetaref(1);

% define el periodo de muestreo
T=.1;

% define parametros del control y calcula las partes ctes

N=20;
lambda=0.1;



% se definen parametros del robot mobil
% teta= orientacion
% x e y coordenadas globales
% A velocidad angular 
% V velocidad lineal
% R radio de la rueda
% no hay retardo

delay=0; 
R=.10;

% 2W distancia entre ruedas
W=.25/2;
V=.3;

% inicializa la variable dg del contador de ptos de la curva de
% referencia en uno

dg=2;
d=0;
kk=10;

% define vectores de largo puntos+d+1
% x,y,teta del robot, xlp prediccion, u control r=ref 


ruido=zeros(1,puntos+d+1);
x=zeros(1,puntos+d+1);
xr=zeros(1,puntos+d+1);
xsal=zeros(1,puntos-2);
ysal=zeros(1,puntos-2);
u=zeros(1,puntos+N+4);
y=zeros(1,puntos+d+1);
yr=zeros(1,puntos+d+1);
teta=zeros(1,puntos+d+1);
dteta=zeros(1,puntos+d+1);
xl=zeros(1,puntos+N+4);
ylp=zeros(1,puntos+N+4);
yl=zeros(1,puntos+d+1);
A=zeros(1,puntos+d+1);
wd=zeros(1,puntos+d+1);
wi=zeros(1,puntos+d+1);

% definicion de la trayectoria en coord. globales

% yy=zeros(1,200);
% xx=zeros(1,200);
% for i=1:200
%  yy(i)=(i-0)*.1;
%  xx(i)=1*yy(i)-.0*yy(i)^2;
% end




% calcula la matriz G para el optimo

G=tril(ones(N,N),0);
for i=1:N-1
   G=G+tril(ones(N,N),-i);
end


% Q=crea_Q(N);

% para este test uso pesos diferentes en teta e y
% rho1 para teta y rho2 para y

rho1=2;
rho2=1;

Q1=[rho1*eye(N);zeros(N,N)];
Q2=[zeros(N,N);rho2*eye(N)];
Qe=[Q1 Q2];


% el control es poderado de forma cte en el horizonte
Qu=eye(N);

% calcula la matriz G para el multivariable

GG=[(V*T)*G;(.5*(T*V)^2)*G];


M=GG'*Qe*GG + lambda*Qu;

% Invierte M multiplica por G' y elije la primer fila

M1=inv(M)*GG';
q=M1(1,:);


% empieza el lazo de control

x(2)=x0;
y(2)=y0;
teta(2)=teta0;
xr(2)=x0;
yr(2)=y0;


for k=3:puntos

% a partir del control (u es gamma) devo calcular el valor de A(k)
% como dteta=u*ds=u*T*V se calcula A=dteta/T;

dteta(k)=u(k-1)*T*V;
A(k-1)=dteta(k)/T;


% aqui se calculan las velocidades wi e wd con A y V 

wd(k-1)=(A(k)*W+V)/R;
wi(k-1)=(-A(k)*W+V)/R;


% se simula el modelo completo del robot mobil
% calcula salida en el proximo instante con la actual
% para simular las perturbacione producidas por las imperfecciones del suelo de 
% hace variar el radio R de la rueda del robot


if k==kk
  dR=0.0*rand;
  Rr=R+dR;
  kk=kk+10;
  Ar(k-1)=(Rr/2*W)*(wd(k-2)-wi(k-2))*(1+0.0*rand);
  Vr=V/(1+dR/R);
else
  Rr=R;
  Ar(k-1)=A(k-1);
  Vr=V;
end




teta(k)=teta(k-1)+Ar(k-1)*T;
if abs(Ar(k-1)) > 0.001 
  xr(k)=xr(k-1)+(Vr/Ar(k-1))*(sin(teta(k))-sin(teta(k-1)));
  yr(k)=yr(k-1)-(Vr/Ar(k-1))*(cos(teta(k))-cos(teta(k-1)));
else
  xr(k)=xr(k-1)+(Vr*T)*cos(teta(k-1));
  yr(k)=yr(k-1)+(Vr*T)*sin(teta(k-1));
end

% se simula la medicion o sea la salida como si no hubiera error 
% las salidas x y e teta son usadas en el control


teta(k)=teta(k-1)+dteta(k);
if abs(A(k-1)) > 0.001 
  x(k)=x(k-1)+(V/A(k-1))*(sin(teta(k))-sin(teta(k-1)));
  y(k)=y(k-1)-(V/A(k-1))*(cos(teta(k))-cos(teta(k-1)));
else
  x(k)=x(k-1)+(V*T)*cos(teta(k-1));
  y(k)=y(k-1)+(V*T)*sin(teta(k-1));
end

% calcula los puntos de prediccion

xe=x(k);
ye=y(k);
tetae=teta(k);


dl=dg;
for m=1:N


% calcula el pto de contacto (xc,yc) usando (xe,ye) y el look-ahead
% constante de valor alfa. d sera el primer elemento del vector xx e
% yy adonde va a buscar ptos para calcular la distancia y hallar el
% par (xc,yc)


    error=-1;
    iii=dl;
    while (error < 0)
       dist=(xe-xx(iii))^2+(ye-yy(iii))^2;
       error=(dist-alfa^2);
       iii=iii+1;
    end
    xc=xx(iii-2);
    yc=yy(iii-2);
    dl=iii-1;
    if m==1
       dg=dl;
    end


    [xs,ys,tetas]=curva(xe,ye,tetae,alfa,xc,yc,T,V);

% transforma para coor locales la referencia

    [xl,yl]=tr_coord(xs,ys,x(k),y(k),teta(k));

    refx(m)=xl;
    refy(m)=yl;
    refteta1(m)=tetas;
    xe=xs;
    ye=ys;
    tetae=tetas;  

end


refyy=refy';
refxx=refx';
refteta=refteta1';


% calculo de la prediccion.Como solo preciso ylp(k) e ylp(k-1)
% calculo 1 paso. CASO OPTIMO NORMAL


if delay==0 
    ylp(k)=0;

    [x12,ylp(k-1)]=tr_coord(x(k-1),y(k-1),x(k),y(k),teta(k));
    tetap(k)=teta(k);
    tetap(k-1)=teta(k-1);
  else
    for i=1:delay
        ylp(k)=yl(k);
        ylp(k-1)=yl(k-1);
        deltau=0;
        
        if (k+i-delay-2)>0
           deltau=(u(k+i-1-delay)-u(k+i-delay-2));
        end         
        if (k+i-delay-2)==0
           deltau=u(k+i-1-delay);
        end 


       ylp(k)=2*yl(k-1)-yl(k-2)+deltau*0.5*(V*T)^2;

     end
  end



% aqui se calcula el control a partir de la referencia y del modelo
% usa GPC. El control se calcula como 
% u=u(k-1) - q*(f - ref)

for i=1:N
  S(i,1)=i+1;
  S(i,2)=-i;
end

S1=[S;zeros(N,2)];

S2=[zeros(N,2);S];

SS=[S1 S2];
f=SS*[tetap(k) tetap(k-1) ylp(k) ylp(k-1)]';

refmult=[refteta;refyy];

u(k)=u(k-1)-q*(f-refmult);

% final del lazo
end

% elimina los ultimos puntos de los vectores

for i=3:puntos
    ysal(i-2)=yr(i);
    xsal(i-2)=xr(i);
    ymed(i-2)=y(i);
    xmed(i-2)=x(i);
    
end


plot(xsal,ysal)
hold on
%plot(xmed,ymed,'r')
plot(xx,yy,'g')
hold off
