clear all;
clc;
tspan = [0 500];
%% this y0 is trim solution
%%y0(1)=Velocity,y0(2)=AOA,y0(3)=pitchrate(q),y0(4)=flightpath angle=1.3099
% y0=[39.5514    0.0872    0.0000    1.3099]
% y0=[41.3358 5*pi/180 0 0]
%y0=[41.4975    0.0872   -0.0000   -0.0872]
y0=[39.5514    0.0872   -0.0000    1.3099]
%y0 = [ 42.2071    0.0873   -0.0000     0]
[t y] = ode45(@fun_trim_wind,tspan,y0);
figure(1)
subplot(1,2,1)
plot(t,y(:,1),'linewidth',3);

set(gca,'fontsize',20)

%title('Trim flight climbing for cessna','FontWeight','bold', 'FontSize',12, 'FontName','Times New Roman')
grid on 

xlabel('Time in seconds')
ylabel('Velocity in m/s')
subplot(1,2,2)
plot(t,y(:,2),'linewidth',3)
set(gca,'fontsize',20)
grid on
xlabel('Time in seconds')
ylabel('AOA in rad')
figure(2)
subplot(1,2,1)
plot(t,y(:,3),'linewidth',3)
set(gca,'fontsize',20)
grid on
xlabel('Time in seconds')
ylabel('Pitch rate in rad/s')
subplot(1,2,2)
plot(t,y(:,4),'linewidth',3)
set(gca,'fontsize',20)
%suptitle('Open loop simulation of longitudinal motion with  perturbations to control surface')

grid on
xlabel('Time in seconds')
ylabel('Flight path angle in rad')

