clear all
clc
alpha=input('Enter angle of attack')
V=input('Enter The required velocity')
alpharad=alpha*pi/180;
g = 9.8;
rho = 1.225;
q=0
m = 1043.26;
C_bar = 1.493;
b = 10.911;
S_ref = 16.1651;


C_L_0 = 0.25;
C_L_alpha = 4.47;
C_L_q = 1.7;
C_L_delta_e = 0.3476;

% Ignore all other C_D derivatives excluding C_D_0
% Use C_D = C_D_0 + k*C_L^2

C_D_0 = 0.036;
k = 0.3;

C_Y_beta = -0.31;
C_Y_p = -0.037;
C_Y_r = 0.21;
C_Y_delta_a = 0;

C_l_beta = -0.089;
C_l_p = -0.47;
C_l_r = 0.096;
C_l_delta_a = -0.09;
C_l_delta_r = 0.0147;

C_M_0 = -0.02;
C_M_alpha = -1.8;
C_M_alpha_dot = -12.4;
C_M_delta_e = -1.28;

C_N_beta = 0.065;
C_N_p = -0.03;
C_N_r = -0.99;
C_N_delta_a = -0.0053;
C_N_delta_r = -0.0657;
delta_e=-( C_M_0 +C_M_alpha*alpharad)/C_M_delta_e;
C_L = C_L_0 + C_L_alpha*alpharad +   C_L_delta_e*delta_e;
C_D = C_D_0 + k*C_L^2;
C_M = C_M_0 + C_M_alpha*alpharad +  C_M_delta_e*delta_e;
C_X = -C_D*cos(alpharad) + C_L*sin(alpharad);
C_Z = -C_D*sin(alpharad) - C_L*cos(alpharad);
cns=-rho*V^2*S_ref*C_Z/(2*m*g);
thetarad=acos(cns);
theta=thetarad*180/pi;
T=m*g*sin(theta*pi/180)-(rho*S_ref*C_X*V^2)/2;
u=sqrt(V^2/(sec(alpharad)*sec(alpharad)));
w=sqrt(V^2-u^2);
fprintf('The required elevator deflection is %i\n',delta_e*180/pi)
fprintf('The pitch angle  is %i\n',theta)
fprintf('The required Thrust is %i\n',T)
save('myfile.mat','alpha','V','T','theta','delta_e')
f1=T/m-g*sin(thetarad)+0.5*rho*V^2*S_ref*C_X/m
f2=.5*rho*V^2*S_ref*C_Z+m*g*cos(thetarad)
f3=0.5*rho*V^2*S_ref*C_bar*C_M
y0 = [u w q theta*pi/180];
[t,y] = ode45(@fun_DE,[0 10],y0)
subplot(2,2,1)
plot(t,y(:,1))
xlabel('Time in s')
ylabel('u in m/s')
subplot(2,2,2)
plot(t,y(:,2))
xlabel('Time in s')
ylabel('w in m/s')
subplot(2,2,3)
plot(t,y(:,3))
xlabel('Time in s')
ylabel('q in rad/s')
subplot(2,2,4)
plot(t,y(:,4))
xlabel('Time in s')
ylabel('theta in rad')


