function [ dydt] =fun_trim_wind_level( t,y )
alpharad=y(2);
g = 9.8;
rho = 1.225;
%q=0
%y(4)=0 % for level flight
m = 1043.26;
C_bar = 1.493;
b = 10.911;
S_ref = 16.1651;
delta_e= -0.1383;
if t==0 && t<=100
 delta_e=  -0.1383;
end
 if t>=101&&t<=111
        delta_e=  -0.1383+(1*pi/180);
    end
    if t>103 && t<=200
        delta_e=-0.1383;
    end
    T=    2.3968e+03

  %T=  2.3968e+03
%T=1.5206e+03

%T=1.2109e+04
%T= 2.4989e+03;
% if t==0 && t<=100
% T=  -2.4989e+03;
% end
%  if t>=101&&t<=111
%         T=  -2.4989e+03*1.1;
%     end
%     if t>103 && t<=200
%        T=-2.4989e+03;
%     end
%  if t>=101&&t<=105
%         T=  1.2109e+04+1*10^4;
%     end
%     if t>103 && t<=200
%         T=1.2109e+04;
%      end


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
I_yy = 1824.93;
%delta_erad=-( C_M_0 +C_M_alpha*alpharad)/C_M_delta_e
C_L = C_L_0 + C_L_alpha*alpharad +   C_L_delta_e*delta_e
C_D = C_D_0 + k*C_L^2
C_M = C_M_0 + C_M_alpha*alpharad +  C_M_delta_e*delta_e
C_X = -C_D*cos(alpharad) + C_L*sin(alpharad)
C_Z = -C_D*sin(alpharad) - C_L*cos(alpharad)

dydt=zeros(6,1)
dydt(1)=1/(m)*(T*cos(y(2))-m*g*sin(y(4))-.5*rho*y(1)^2*S_ref*C_D);
dydt(2)=y(3)-(-g/y(1)+.5*rho*y(1)^2*S_ref*C_L/(m*y(1))+T*sin(y(2))/(m*y(1)));
dydt(3)=1/(2*I_yy)*rho*(y(1)^2)*S_ref*C_bar*C_M
dydt(4)=y(3);
dydt(5)=y(1)*cos(y(4));
dydt(6)=-y(1)*sin(y(4))
end

