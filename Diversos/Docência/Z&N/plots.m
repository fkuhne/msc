% laboratorio 7

close all;
clear all;
clc;

load temperatura.dat;
load potencia.dat;

hold on;
grid on;
box on;

plot(temperatura(:,1),temperatura(:,3),'r');
plot(potencia(:,1),potencia(:,3));
axis([-100 550 -20 140]);
xlabel('tempo (s).3/6
');
ylabel('temperatura (verm) e potencia (azul)');