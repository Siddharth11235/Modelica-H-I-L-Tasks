function [ dydt ] =fun_DE( t,y)

 V=35;
 
g = 9.8;
rho = 1.225;
%q=0
m = 1043.26;
C_bar = 1.493;
b = 10.911;
S_ref = 16.1651;
alpharad=atan(y(2)/y(1));
C_L_0 = 0.25;
C_L_alpha = 4.47;
C_L_q = 1.7;
C_L_delta_e = 0.3476;

% Ignore all other C_D derivatives excluding C_D_0
% Use C_D = C_D_0 + k*C_L^2

C_D_0 = 0.036;
k = 0.3;


C_M_0 = -0.02;
C_M_alpha = -1.8;
C_M_alpha_dot = -12.4;
C_M_delta_e = -1.28;
delta_erad=-0.1383;
T=8235.5;
 %delta_erad=-( C_M_0 +C_M_alpha*atan(y(2)/y(1)))/C_M_delta_e
%delta_erad=-0.1383+(pi/180)
% if t>=0 && t<=100
% delta_erad=delta_erad
% end
% if t>100 && t<=101
%     delta_erad=-0.1383+0.25*(pi/180)
% end
% if t>101
%     delta_erad=-0.1383
% end
C_M=C_M_0+C_M_alpha*atan(y(2)/y(1))+C_M_delta_e*delta_erad;

C_L = C_L_0 + C_L_alpha*atan(y(2)/y(1)) +   C_L_delta_e*delta_erad;
C_D = C_D_0 + k*C_L^2;
%C_M = C_M_0 + C_M_alpha*alpharad +  C_M_delta_e*delta_e
C_X = -C_D*cos(atan(y(2)/y(1))) + C_L*sin(atan(y(2)/y(1)));
C_Z = -C_D*sin(atan(y(2)/y(1))) - C_L*cos(atan(y(2)/y(1)));
I_yy = 1824.93;

%y(4)=acos(cns)
%theta=thetarad*180/pi


dydt=zeros(4,1);
dydt(1)=1/(2*m)*(rho*(y(1)^2+y(2)^2)*S_ref*C_X)+(T/m)-g*sin(y(4));
dydt(2)=1/(2*m)*(rho*(y(1)^2+y(2)^2)*S_ref*C_Z)+g*cos(y(4));
dydt(3)=1/(2*I_yy)*rho*(y(1)^2+y(2)^2)*S_ref*C_bar*C_M;
dydt(4)=y(3)
%subplot(2,2,1)
%plot(t,y(:,1),'linewidth',6)

end

