function [y, dxdt] = sim(x,u)
V=x(1);
Alpha=x(2);
Beta=x(3);
xe=x(4);
ye=x(5);
ze=x(6);
p=x(7);
q=x(8);
r=x(9);
mu=x(10);
Gamma=x(11);
chi=x(12);
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
C_L_Alpha = 4.47;
C_L_q = 1.7;
C_L_delta_e = 0.3476;
delta_e= u(2);
  T= u(1);
  delta_a=0;
  delta_r=0;
  Alpha_dot=0;

C_D_0 = 0.036;
k = 0.3;
// C_D_Alpha = 0.13;
// C_D_Beta = 0.17;
// C_D_delta_e = 0.06;

C_Y_Beta = -0.31;
C_Y_p = -0.037;
C_Y_r = 0.21;
C_Y_delta_a = 0;

C_l_Beta = -0.089;
C_l_p = -0.47;
C_l_r = 0.096;
C_l_delta_a = -0.09;
C_l_delta_r = 0.0147;

C_M_0 = -0.02;
C_M_Alpha = -1.8;
C_M_Alpha_dot = -12.4;
C_M_delta_e = -1.28;

C_N_Beta = 0.065;
C_N_p = -0.03;
C_N_r = -0.99;
C_N_delta_a = -0.0053;
C_N_delta_r = -0.0657;
C_L = C_L_0 + C_L_Alpha*Alpha + C_L_q*(q*C_bar/(2*V)) + C_L_delta_e*delta_e;
C_D = C_D_0 + k*C_L^2;
C_Y = C_Y_Beta*Beta + C_Y_p*(p*b/(2*V)) + C_Y_r*(r*b/(2*V)) + C_Y_delta_a*delta_a;

C_l = C_l_Beta*Beta + C_l_p*(p*b/(2*V)) + C_l_r*(r*b/(2*V)) + C_l_delta_a*delta_a + C_l_delta_r*delta_r;
C_M = C_M_0 + C_M_Alpha*Alpha + C_M_Alpha_dot*(Alpha_dot*C_bar/(2*V)) + C_M_delta_e*delta_e;
C_N = C_N_Beta*Beta + C_N_p*(p*b/(2*V)) + C_N_r*(r*b/(2*V)) + C_N_delta_a*delta_a + C_N_delta_r*delta_r;

C_X = -C_D*cos(Alpha) + C_L*sin(Alpha);
C_Z = -C_D*sin(Alpha) - C_L*cos(Alpha);
dxdt=zeros(12,1)
dxdt(1)=1/m*(T*cos(Alpha)*cos(Beta)-0.5*rho*V^2*S_ref*(C_D*cos(Beta)-C_Y*sin(Beta))-m*g*sin(Gamma));
dxdt(2)=q-1/cos(Beta)*((p*cos(Alpha)+r*sin(Alpha))*sin(Beta)-g/V*cos(Gamma)*cos(mu)+0.5*rho*V^2*S_ref*C_L/(m*V)+T*sin(Alpha)/(m*V));
dxdt(3)=(p*sin(Alpha)-r*cos(Alpha))+1/(m*V)*(-T*cos(Alpha)*sin(Beta)+0.5*rho*V^2*S_ref*(C_Y*cos(Beta)+C_D*sin(Beta))+m*g*cos(Gamma)*sin(mu))
dxdt(4)=V*cos(Gamma)*cos(chi);
dxdt(5)=V*cos(Gamma)*sin(chi);
dxdt(6)=-V*sin(Gamma);
dxdt(7)=(I_yy-I_zz)/I_xx*q*r+1/(2*I_xx)*rho*V^2*S_ref*b*C_l;
dxdt(8)=(I_zz-I_xx)/I_yy*p*r+1/(2*I_yy)*rho*V^2*S_ref*C_bar*C_M;
dxdt(9)=(I_xx-I_yy)/I_zz*p*q+1/(2*I_zz)*rho*V^2*S_ref*b*C_N;
dxdt(10)=p+tan(Gamma)*sin(mu)*q+tan(Gamma)*cos(mu)*r;
dxdt(11)=cos(mu)*q-sin(mu)*r;
dxdt(12)=sec(Gamma)*sin(mu)*q+sec(Gamma)*cos(mu)*r;

y = [q;Alpha];
endfunction
