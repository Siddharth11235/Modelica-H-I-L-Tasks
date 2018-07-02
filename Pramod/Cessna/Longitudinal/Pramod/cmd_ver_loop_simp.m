tspan=[0 10];
y0=[pi/4 100 100];
[t y]=ode45(@(t,y) ver_loop_simp(t,y),tspan,y0);
figure(1)

subplot(2,2,1)
plot(t,y(:,2),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Xe in m')
set(gca,'fontsize',30)
subplot(2,2,2)
plot(t,y(:,3),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Xe in m')
set(gca,'fontsize',30)
subplot(2,2,3)
plot(y(:,3),y(:,2),'linewidth',3)
grid on
xlabel('Ze in m')
ylabel('Xe in m')
axis square;
xlim([0 120]);
ylim([0 120]);
set(gca,'fontsize',30)
daspect([1 1 1])
