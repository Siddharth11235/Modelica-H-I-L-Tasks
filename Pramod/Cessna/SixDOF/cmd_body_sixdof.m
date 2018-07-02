clear all;
clc;
u=41.1470;
w= 3.5999;
theta=    pi/180;
%y0=[u v w xe ye ze p q r phi theta psi];
y0=[u 0 w 600 0 -1200 0 0 0 0 theta 0];
tspan = [0 10];
[t y] = ode45(@fun_sixdof_body,tspan,y0);
subplot(3,4,1)
%plot(t,y(:,1))
plot(t,y(:,1),'linewidth',3); 
grid on 
title('longitudinal motion trim')
xlabel('Time in seconds')
ylabel('Horizantal body in m/s')
subplot(3,4,2)
plot(t,y(:,2),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Side velocity in m/s')
subplot(3,4,3)
%plot(t,y(:,3))
plot(t,y(:,3),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Vertical velocity in m/s')
subplot(3,4,4)
plot(t,y(:,4),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Xe in m')
subplot(3,4,5)
plot(t,y(:,5),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Ye in m')
subplot(3,4,6)
plot(t,y(:,6),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Ze in m')
grid on
subplot(3,4,7)
plot(t,y(:,7),'linewidth',3)
xlabel('Time in seconds')
ylabel('Roll rate in rad/s')
grid on
subplot(3,4,8)
plot(t,y(:,8),'linewidth',3)
xlabel('Time in seconds')
ylabel('Pitch rate in rad/s')  
grid on
subplot(3,4,9)
plot(t,y(:,9),'linewidth',3)
xlabel('Time in seconds')
ylabel('Yaw rate in rad/s') 
grid on
subplot(3,4,10)
plot(t,y(:,10),'linewidth',3)
xlabel('Time in seconds')
ylabel('roll in rad') 
grid on
subplot(3,4,11)
plot(t,y(:,11),'linewidth',3)
xlabel('Time in seconds')
ylabel('pitch angle in rad') 
grid on
subplot(3,4,12)
plot(t,y(:,12),'linewidth',3)
xlabel('Time in seconds')
ylabel('yaw angle in rad') 
grid on