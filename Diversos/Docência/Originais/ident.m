
% sistema de primeira ordem
K=3;
tau=2.5;
G1=tf(K,[tau 1]);

% sistema de segunda ordem sobre-amortecido
K=0.9;
p1=1/4;
p2=1/3;
p3=1/0.3;
G2=zpk([],[-p1 -p2 -p3],K);

% sistema de segunda ordem sub-amortecido
K=7;
xi=0.5;
wn=15;
G3=tf(K*wn*wn,[1 2*xi*wn wn^2]);

figure(1);
subplot(311);
[y1,t1]=step(G1);
step(G1);
xlabel('');

subplot(312);
[y2,t2]=step(G2);
step(G2);
title('');
xlabel('');

subplot(313);
[y3,t3]=step(G3);
step(G3);
title('');



%ens1=[t1,y1];
%ens2=[t2,y2];
%ens3=[t3,y3];

%save 'ensaio1.txt' ens1 -ASCII
%save 'ensaio2.txt' ens2 -ASCII
%save 'ensaio3.txt' ens3 -ASCII









