close all; clear all;

hold on; box on; grid on;

load vref;
load wref;
Kf = size(vref,2);

load fvalconst;
plot(fval);

load fvalsat;
plot(fval,'-.r');

hold off;

figure;
hold on; box on; grid on;

load xconst;
load yconst;
plot(x,y);

load xsat;
load ysat;
plot(x,y,'-.r');

hold off;

figure;
hold on; box on; grid on;
load vconst;
subplot(2,1,1);
hold on; box on; grid on;
stairs(v,'r'); stairs(vref,'-.k');
load vsat;
stairs(v,':');
legend('v','v_{ref}',1);
ylabel('v');
axis([0 Kf 0.29 0.53]);
hold off;
subplot(2,1,2);
load wconst;
hold on; box on; grid on;
stairs(w,'r'); stairs(wref,'-.k');
load wsat;
stairs(w,':');
legend('w','w_{ref}',1);
ylabel('w');
xlabel('samples');
axis([0 Kf -5.4 0.2]);
hold off;

