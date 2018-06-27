clear all;
clc;
syms a e c d f h
%[a=V,e=alpha,c=q,d=gamma, f=delta_e,h=T]
% V=a;
% alpha=e;
% q=c;
% gamma=d;
g = 9.8;
rho = 1.225;
%q=0
y0 = [  41.4146 5*pi/180 0 -2.5*pi/180 100 -200 ];
% delta_e=f;
% T=g;
  delta_e=  -0.1383;
  T=1.2109e+04;
m = 1043.26;
C_bar = 1.493;
b = 10.911;
S_ref = 16.1651;
C_L_0 = 0.25;
C_L_alpha = 4.47;
C_L_q = 1.7;
C_L_delta_e = 0.3476;
C_D_0 = 0.036;
k = 0.3;
I_yy = 1824.93;
C_M_0 = -0.02;
C_M_alpha = -1.8;
C_M_alpha_dot = -12.4;
C_M_delta_e = -1.28;
C_L=C_L_0+C_L_alpha*e+C_L_delta_e*f;
C_D=C_D_0+k*C_L^2;
C_M=C_M_0+C_M_alpha*e+C_M_delta_e*f;
C_X=-C_D*cos(e)+C_L*sin(e);
C_Z=-C_D*sin(e)-C_L*sin(e);
f1=1/m*(h*cos(e)-0.5*rho*a^2*S_ref*C_D-m*g*sin(d));
f2=c-(-g/a+0.5*rho*a^2*S_ref*C_L/(m*a)+(h*sin(e)/(m*a)));
f3=1/(2*I_yy)*rho*a^2*S_ref*C_bar*C_M;
f4=c;
ans=jacobian([f1,f2,f3,f4],[a,e,c,d,f,h])

ans1=subs(ans,{a,e,c,d,f,h},{y0})
%lin_mod=vpa(ans1)
sys_mat(:,1)=ans1(:,1);
sys_mat(:,2)=ans1(:,2);
sys_mat(:,3)=ans1(:,3);
sys_mat(:,4)=ans1(:,4);
in_mat(:,1)=ans1(:,5);
in_mat(:,2)=ans1(:,6);
A = double(sys_mat);
B=double(in_mat);
C=diag([1,1,1,1])
D=[0;0;7.2;0];
disp(sys_mat);
disp(in_mat);
%sys=ss(A,B,C,0)
[eig_vect,eig_value]=eig(A)
p=[ -0.1856 + 4.6405i;
  -0.1856 - 4.6405i;
  -0.0524 + 0.1614i;
  -0.0524 - 0.1614i];
l=place(A,B,p)
eig(A-B*l)
eig(A)
t11=88;
t22=0.1
%t11=roots([B(1,1) -2 3*B(2,2)])
%t22=3/t11(1)
t12=0;
t21=0;
%F=1/(B(1,1)*B(2,2)-B(2,1)*B(1,2))*([A(2,1)*B(1,2)-A(1,1)*B(2,2) A(2,2)*B(1,2)-A(1,2)*B(2,2) B(2,1)*A(2,3) -A(1,4)*B(2,1);A(1,1)*B(2,1)-A(2,1)*B(1,1) A(1,2)*B(2,1)-A(2,2)*B(1,2) -B(1,1)*A(2,3)  A(1,4)*B(2,1)])+[t11 t12 0 0;t21 t22 0 0]
%F=1/(B(1,1)*B(2,2)-B(2,1)*B(1,2))*([A(2,1)*B(1,2)-A(1,1)*B(2,2) A(2,2)*B(1,2)-A(1,2)*B(2,2) B(1,2)*A(2,3) -A(1,4)*B(2,2);A(1,1)*B(2,1)-A(2,1)*B(1,1) A(1,2)*B(2,1)-A(2,2)*B(1,1) -B(1,1)*A(2,3)  A(1,4)*B(2,1)])+[t11 t12 0 0;t21 t22 0 0]
%eig(A+B*F)
%charpoly(A+B*F)
Asp=[A(2,2) A(2,3);A(3,2) A(3,3)];
Bsp=[B(2,1) B(2,2);B(3,1) B(3,2)];
Bspu=Bsp(:,1)
p=[-.07+0.64i -.07-0.64i];
i=place(Asp,Bspu,p)
Apg=[A(1,1) A(1,4);A(4,1) A(4,4)]
Bpg=[B(1,1) B(1,2);B(4,1) B(4,2)]


