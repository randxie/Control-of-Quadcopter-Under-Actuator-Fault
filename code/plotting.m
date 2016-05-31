% for plotting
t = t(1:40000);
distance = distance(1:40000);
distanceAdapt = distanceAdapt(1:40000);
plot(t,distance,'b', 'linewidth', 2);
hold on;
plot(t,distanceAdapt,'k:', 'linewidth', 2);
legend('no adaptation', 'with adaptation');
xlabel('Time (s)');
ylabel('3D Tracking Error');
title('Fault Induced in 100s');

time2 = linspace(0,250,size(kLMonitor,1));
plot(time2,kLMonitor(:,1),'r:','linewidth', 2)
hold on;
plot(time2,kLMonitor(:,2),'k:','linewidth', 2)
hold on;
plot(time2,kLMonitor(:,3),'g:','linewidth', 2)
hold on;
plot(time2,kLMonitor(:,4),'b:','linewidth', 2)
hold on;

plot(time2,kLMonitor(:,1),'r:','linewidth', 2)
hold on;
plot(time2,(time2<=100)*0.6125 * 10^-4 + 0.6125 * 10^-4*0.5*(time2>100),'k--','linewidth', 2 )
xlabel('Time(s)');
ylabel('Motor 1 Parameter (kL1)');
legend('Estimtaed','Real');
title('Parameter Tracking');