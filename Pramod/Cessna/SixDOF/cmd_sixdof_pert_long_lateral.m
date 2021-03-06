clear all;
clc;
tspan = [0 1000];
%y0=[V alpha beta xe ye ze p q r mu gamma chi]
%y0=[ 41.305419934287598948685626965016, 0.087235425492026424931779615690175, 0, 8359.8250197566503629786893725395, 0, -344.69006447241991963892360217869, 0, 0.00000065604114286579631879555308399121, 0, 0, 0.017516460062603447789264521361474, 0]
 
y0=[ 41.305419061789173440502054290846 0.0872355555554296391296276169669 0  100 0  -200 0 0 0 0 -pi/180 0]
[t y] = ode45(@fun_sixdof_pert,tspan,y0);
figure(1);
subplot(1,2,1)
%plot(t,y(:,1))
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
set(gca,'fontsize',20)
figure(2)
subplot(1,2,1)
plot(t,y(:,4),'linewidth',3)
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
figure(3)
subplot(1,2,1)
plot(t,y(:,8),'linewidth',3)
xlabel('Time in seconds')
ylabel('Pitch rate in rad/s') 
set(gca,'fontsize',30)
grid on
subplot(1,2,2)
plot(t,y(:,11),'linewidth',3)
xlabel('Time in seconds')
ylabel('Flight path angle in rad') 
set(gca,'fontsize',30)
grid on;
figure(4);
subplot(1,2,1)
%plot(t,y(:,3))
plot(t,y(:,3),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Beta in rad')

set(gca,'fontsize',30)
subplot(1,2,2)
plot(t,y(:,5),'linewidth',3)
grid on
xlabel('Time in seconds')
ylabel('Ye in m')
set(gca,'fontsize',30)
figure(5)
grid on
subplot(1,2,1)
plot(t,y(:,7),'linewidth',3)
xlabel('Time in seconds')
ylabel('Roll rate in rad/s')
set(gca,'fontsize',30)
grid on
subplot(1,2,2)
plot(t,y(:,9),'linewidth',3)
xlabel('Time in seconds')
ylabel('Yaw rate in rad/s') 
set(gca,'fontsize',30)
grid on
figure(6)
subplot(1,2,1)
plot(t,y(:,10),'linewidth',3)
xlabel('Time in seconds')
ylabel('Mu in rad') 
set(gca,'fontsize',30)
grid on
subplot(1,2,2)
plot(t,y(:,12),'linewidth',3)
xlabel('Time in seconds')
ylabel('Chi angle in rad') 
set(gca,'fontsize',30)
grid on
% g = 9.8;
% rho = 1.225;
% 
% m = 1043.26;
% C_bar = 1.493;
% b = 10.911;
% S_ref = 16.1651;
% 
% I_xx = 1285.31;
% I_yy = 1824.93;
% I_zz = 2666.893;
% I_xz = 0;
% I_yz = 0;
% I_xy = 0;
% 
% C_L_0 = 0.25;
% C_L_alpha = 4.47;
% C_L_q = 1.7;
% C_L_delta_e = 0.3476;
% 
% C_D_0 = 0.036;
% k = 0.3;
% % C_D_alpha = 0.13;
% % C_D_beta = 0.17;
% % C_D_delta_e = 0.06;
% 
% C_Y_beta = -0.31;
% C_Y_p = -0.037;
% C_Y_r = 0.21;
% C_Y_delta_a = 0;
% 
% C_l_beta = -0.089;
% C_l_p = -0.47;
% C_l_r = 0.096;
% C_l_delta_a = -0.09;
% C_l_delta_r = 0.0147;
% 
% C_M_0 = -0.02;
% C_M_alpha = -1.8;
% C_M_alpha_dot = -12.4;
% C_M_delta_e = -1.28;
% 
% C_N_beta = 0.065;
% C_N_p = -0.03;
% C_N_r = -0.99;
% C_N_delta_a = -0.0053;
% C_N_delta_r = -0.0657;
