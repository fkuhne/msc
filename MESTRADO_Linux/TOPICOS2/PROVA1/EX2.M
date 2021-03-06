clear all;
close all;

A = [1.2272 1 ; -0.3069 0];
B = [0.0634 ; 0.0978];
C = [1 0];
D = 0;

u = 1; %Degrau unitario

% CI:
x(:,1) = [0 ; 0];
x_em(:,1) = [1 ; 1];
P_m = 0.001;

% Vari�ncias:
Q = 0.01*eye(2); 
R = 0.04;

for k = 1 : 50
   x(:,k+1) = A*x(:,k) + B*u + sqrt(Q)*randn(2,1);
   y(k) = C*x(:,k) + sqrt(R)*randn;
 
   Pinv = inv(P_m) + C'*inv(R)*C;
   K = inv(Pinv)*C'*inv(R);
   vec(:,k) = diag(inv(Pinv));
   
   x_e(:,k) = x_em(:,k) + K*(y(k) - C*x_em(:,k));
   
   x_em(:,k+1) = A*x_e(:,k) + B*u;
   P_m = A*inv(Pinv)*A' + Q;
end

subplot(2,2,1);
hold;
plot(1:50, x(1,1:50), 'kd', 'MarkerSize', 4);
plot(1:50, x(2,1:50), 'k^', 'MarkerSize', 4);
title('estados x_1 e x_2');
hold;

subplot(2,2,2)
plot(1:50,y, 'kd', 'MarkerSize', 4);
title('saida y(t) = x_1(t)');

subplot(2,2,3);
hold;
plot(1:50, x_e(1,1:50), 'kd', 'MarkerSize', 4);
plot(1:50, x_e(2,1:50), 'k^', 'MarkerSize', 4);
title('estados x_1 e x_2');
title('estima�ao dos estados');
hold

subplot(2,2,4)
hold;
plot(1:50,vec(1,1:50), 'k');
plot(1:50,vec(2,1:50), 'k');
title('diagonal da matriz de covariancia P(k|k)');
hold;
