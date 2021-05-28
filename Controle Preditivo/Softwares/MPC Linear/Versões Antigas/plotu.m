load vref;
load wref;

figure('name','Controles v e w','numbertitle','off');

subplot(2,1,1);
hold on; box on; grid on;

plot(vref,'-.k');
 
load vc;
plot(v,'r');

legend('v_{ref}','v',1);

load vu;
plot(v,':k');

axis([0 838 0.17 0.51]);

ylabel('v');
% title('Controle');
hold off;

subplot(2,1,2);
hold on; box on; grid on;

plot(wref,'-.k');

load wc;
plot(w,'r');

legend('w_{ref}','w',1);

load wu;
plot(w,':k');

axis([0 838 -0.56 0.2]);

ylabel('w');
xlabel('samples');
hold off;
