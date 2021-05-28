clear all;
close all;

A = [1.2272 1 ; -0.3069 0];
B = [0.0634 ; 0.0978];
C = [1 0];
D = 0;
Q = 0.01*eye(2); 
R = 0.04;
u = 1; %Degrau unitario

% CI:
x(:,1) = [0 ; 0];
x_em1(:,1) = [rand ; rand];
P_m = eye(2)*1000;

for k = 1 : 50
   x(:,k+1) = A*x(:,k) + B*u + sqrt(Q)*randn(2,1);
   y(k) = C*x(:,k) + sqrt(R)*randn;

   K = P_m*C'*inv(C*P_m*C' + R);
   P = (eye(2) - K*C)*P_m;
   x_e1(:,k) = x_em1(:,k) + K*(y(k) - C*x_em1(:,k));
   x_em1(:,k+1) = A*x_e1(:,k) + B*u;
   P_m = A*P*A' + Q;
end


% CI:
x_em2(:,1) = [rand ; rand];
P_m = eye(2)*1;

for k = 1 : 50
   x(:,k+1) = A*x(:,k) + B*u + sqrt(Q)*randn(2,1);
   y(k) = C*x(:,k) + sqrt(R)*randn;
 
   Pinv = inv(P_m) + C'*inv(R)*C;
   K = inv(Pinv)*C'*inv(R);
   x_e2(:,k) = x_em2(:,k) + K*(y(k) - C*x_em2(:,k));
   x_em2(:,k+1) = A*x_e2(:,k) + B*u;
   P_m = A*inv(Pinv)*A' + Q;
end


% CI:
z(:,1) = [0 ; 0];
z_em(:,1) = [rand ; rand];
Z_m = eye(2)*1;

for k = 1 : 50
   x(:,k+1) = A*x(:,k) + B*u + sqrt(Q)*randn(2,1);
   y(k) = C*x(:,k) + sqrt(R)*randn;
 
   Z = Z_m + C'*inv(R)*C;
   z_e(:,k) = z_em(:,k) + C'*inv(R)*y(k);
   x_e3(:,k) = inv(Z)*z_e(:,k);
   Z_m = inv(A*inv(Z)*A' + Q);
   z_em(:,k+1) = Z_m*A*inv(Z)*z_e(:,k);
end

hold;
plot(1:50,x(1,1:50), 'r');
plot(1:50,x(2,1:50), 'r');

plot(1:50,x_e1(1,1:50), 'b');
plot(1:50,x_e1(2,1:50), 'b');

plot(1:50,x_e2(1,1:50), '--c');
plot(1:50,x_e2(2,1:50), '--c');

plot(1:50,x_e3(1,1:50), '-.k');
plot(1:50,x_e3(2,1:50), '-.k');
hold;

figure;
subplot(3,1,1);
plot(1:50,x_e1(:,1:50) - x_e2(:,1:50), 'k');
title('Grafico 1');

subplot(3,1,2);
plot(1:50,x_e1(:,1:50) - x_e3(:,1:50), '--k');
title('Grafico 2');

subplot(3,1,3);
plot(1:50,x_e2(:,1:50) - x_e3(:,1:50), '-.k');
title('Grafico 3');

