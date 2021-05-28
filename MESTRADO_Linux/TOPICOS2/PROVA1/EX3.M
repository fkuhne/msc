clear all;
close all;

A = [1.2272 1 ; -0.3069 0];
B = [0.0634 ; 0.0978];
C = [1 0];
D = 0;

u = 1; %Degrau unitario

% CI:
x(:,1) = [0 ; 0];
z(:,1) = [0 ; 0];
z_em(:,1) = [0 ; 0]; % z_em = z_e(k|k-1)
Z_m = eye(2)*1; % Z_m = Z(k|k-1)

% Variâncias:
Q = 0.01*eye(2); 
R = 0.04;

for k = 1 : 50
   x(:,k+1) = A*x(:,k) + B*u + sqrt(Q)*randn(2,1);
   y(k) = C*x(:,k) + sqrt(R)*randn;
   
   Z = Z_m + C'*inv(R)*C; % Z(k|k) = Z(k|k-1) + I ;  I = C' * inv(R) * C
   vec(:,k) = diag(inv(Z));
   
   % Estimação:
   z_e(:,k) = z_em(:,k) + C'*inv(R)*y(k); % zhat(k|k) = zhat(k|k-1) + i ;  i = C' * inv(R) * y(k)
   x_e(:,k) = inv(Z)*z_e(:,k);    
   
   % Predição:
   Z_m = inv(A*inv(Z)*A' + Q); % Z(k+1|k) = inv(A * inv(Z) * A' + Q)
   z_em(:,k+1) = Z_m*A*inv(Z)*z_e(:,k); % zhat(k+1|k) = L(k+1|k) * y(k) ;  L(k+1|k) = Z(k+1|k) * A * inv(Z(k|k))
end

subplot(2,2,1);
hold;
plot(1:50, x(1,1:50), 'kd', 'MarkerSize', 4);
plot(1:50, x(2,1:50), 'k^', 'MarkerSize', 4);
title('estados x_1 e x_2');
hold;

subplot(2,2,2)
plot(1:50,y, 'kd', 'MarkerSize', 4);
title('saida y(t) = x_1(t) + v(t)');

subplot(2,2,3);
hold;
plot(1:50, x_e(1,1:50), 'kd', 'MarkerSize', 4);
plot(1:50, x_e(2,1:50), 'k^', 'MarkerSize', 4);
title('estimaçao dos estados');
hold

subplot(2,2,4)
hold;
plot(1:50,vec(1,1:50), 'k');
plot(1:50,vec(2,1:50), 'k');
title('diagonal da matriz de covariancia P(k|k)');
hold;
