% MPC utilizando as funcoes do MATLAB   
% e o modelo do julio.

close all;
clear all;
clc;

v = 10; % velocidade tangencial
T = 0.1; % periodo amostral

d = 0.0149; % restricao no controle

% modelo em ss:
A = eye(2);
B = [(v*T)^2 ; v*T];
C = eye(2);
D = [0 ; 0];

% modelo da planta em mod:
pmod = ss2mod(A,B,C,D);

% modelo interno para predicao dos estados:
imod = pmod;

% Horizonte de predicao e controle:
P = 10;
M = P;

% Matrizes de peso:
% do estado:
Q = [];
% e do controle:
R = 1;

% Ganho do controlador sem restricoes:
Ks = smpccon(imod,Q,R,M,P);

% Carrega trajetoria de referencia (setpoint variavel):
load y;
load theta;
r = [theta ; theta]';
tend = 838; % tempo total de simulacao

% Restricao do controle:
ulim = [-d d inf];

[ybar,ubar,ym] = smpcsim(pmod,imod,Ks,tend,r,ulim);

plotall(ybar,ubar);

figure; hold on; box on; grid on;
plot(theta);
plot(ybar);
