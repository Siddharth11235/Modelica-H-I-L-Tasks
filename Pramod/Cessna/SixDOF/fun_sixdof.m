function [dydt ] = fun_sixdof( t,y )
V=y(1);
alpha=y(2);
beta=y(3);
xe=y(4);
ye=y(5);
ze=y(6);
p=y(7);
q=y(8);
r=y(9);
mu=y(10);
gamma=y(11);
chi=y(12);
g = 9.8;
rho = 1.225;

m = 1043.26;
C_bar = 1.493;
b = 10.911;
S_ref = 16.1651;

I_xx = 1285.31;
I_yy = 1824.93;
I_zz = 2666.893;
I_xz = 0;
I_yz = 0;
I_xy = 0;

C_L_0 = 0.25;
C_L_alpha = 4.47;
C_L_q = 1.7;
C_L_delta_e = 0.3476;
delta_e= -0.1383;
  T= 2.5723e+03;
  delta_a=0;
  delta_r=0;
  alpha_dot=0;

C_D_0 = 0.036;
k = 0.3;
% C_D_alpha = 0.13;
% C_D_beta = 0.17;
% C_D_delta_e = 0.06;

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
C_L = C_L_0 + C_L_alpha*alpha + C_L_q*(q*C_bar/(2*V)) + C_L_delta_e*delta_e;
C_D = C_D_0 + k*C_L^2;
C_Y = C_Y_beta*beta + C_Y_p*(p*b/(2*V)) + C_Y_r*(r*b/(2*V)) + C_Y_delta_a*delta_a;

C_l = C_l_beta*beta + C_l_p*(p*b/(2*V)) + C_l_r*(r*b/(2*V)) + C_l_delta_a*delta_a + C_l_delta_r*delta_r;
C_M = C_M_0 + C_M_alpha*alpha + C_M_alpha_dot*(alpha_dot*C_bar/(2*V)) + C_M_delta_e*delta_e;
C_N = C_N_beta*beta + C_N_p*(p*b/(2*V)) + C_N_r*(r*b/(2*V)) + C_N_delta_a*delta_a + C_N_delta_r*delta_r;

C_X = -C_D*cos(alpha) + C_L*sin(alpha);
C_Z = -C_D*sin(alpha) - C_L*cos(alpha);
dydt=zeros(12,1)
dydt(1)=1/m*(T*cos(alpha)*cos(beta)-0.5*rho*V^2*S_ref*(C_D*cos(beta)-C_Y*sin(beta))-m*g*sin(gamma));
dydt(2)=q-1/cos(beta)*((p*cos(alpha)+r*sin(alpha))*sin(beta)-g/V*cos(gamma)*cos(mu)+0.5*rho*V^2*S_ref*C_L/(m*V)+T*sin(alpha)/(m*V));
dydt(3)=(p*sin(alpha)-r*cos(alpha))+1/(m*V)*(-T*cos(alpha)*sin(beta)+0.5*rho*V^2*S_ref*(C_Y*cos(beta)+C_D*sin(beta))+m*g*cos(gamma)*sin(mu))
dydt(4)=V*cos(gamma)*cos(chi);
dydt(5)=V*cos(gamma)*sin(chi);
dydt(6)=-V*sin(gamma);
dydt(7)=(I_yy-I_zz)/I_xx*q*r+1/(2*I_xx)*rho*V^2*S_ref*b*C_l;
dydt(8)=(I_zz-I_xx)/I_yy*p*r+1/(2*I_yy)*rho*V^2*S_ref*C_bar*C_M;
dydt(9)=(I_xx-I_yy)/I_zz*p*q+1/(2*I_zz)*rho*V^2*S_ref*b*C_N;
dydt(10)=p+tan(gamma)*sin(mu)*q+tan(gamma)*cos(mu)*r;
dydt(11)=cos(mu)*q-sin(mu)*r;
dydt(12)=sec(gamma)*sin(mu)*q+sec(gamma)*cos(mu)*r;
end

