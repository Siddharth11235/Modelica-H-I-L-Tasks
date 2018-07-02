clear all;
clc;
tspan = [0 100];
%% this y0 is trim solution
%%y0(1)=Velocity,y0(2)=AOA,y0(3)=pitchrate(q),y0(4)=flightpath angle=1.3099
% y0=[39.5514    0.0872    0.0000    1.3099]

 y0=[          41.4146 5*pi/180 0 -2.5*pi/180 100 -200]

%y0=[ 41.3105 0.0872 0 0 100 -200]
%y0=[39.5514    0.0872   -0.0000    1.3099]
%y0 = [ 42.2071    0.0873   -0.0000     0]
[t y] = ode45(@fun_trim_wind_desent,tspan,y0);
figure(1)
subplot(1,2,1)
plot(t,y(:,1),'linewidth',3); 
grid on 
%title('longitudinal motion trim')
xlabel('Time in seconds')
ylabel('Velocity in m/s')
set(gca,'fontsize',30)

subplot(1,2,2)
plot(t,y(:,2),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('AOA in rad')
set(gca,'fontsize',30)
figure(2)
subplot(1,2,1)
plot(t,y(:,3),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Pitch rate in rad/s')
set(gca,'fontsize',30)
subplot(1,2,2)
plot(t,y(:,4),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Flight path angle in rad')
set(gca,'fontsize',30)
figure(3)
subplot(1,2,1)
plot(t,y(:,5),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Xe in m')
set(gca,'fontsize',30)
subplot(1,2,2)
plot(t,y(:,6),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Ze in m')
set(gca,'fontsize',30)
grid on

