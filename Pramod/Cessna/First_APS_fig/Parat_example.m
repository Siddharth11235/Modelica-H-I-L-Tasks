clear all
clc
A=[2 3;7 9];
b=[2;6]
T=[b A*b];
x_0=[9;6];
x_1=[4;7];
z_0=inv(T)*x_0;
z_1=inv(T)*x_1;
t_init=0
t_fin=1
A_sol=[1 0 0 0;1 1 1 1;-11 1 0 0;-11 -10 -9 -8  ];
%B_sol=[x0(1);xf(1);x0(2);xf(2)];
b_sol=[z_0(2);z_1(2);z_0(1);z_1(1)]

vect=inv(A_sol)*b_sol
syms t
z2=vect(1)+vect(2)*t+vect(3)*t^2+vect(4)*t^3;
figure(1)
subplot(1,2,1)
h=ezplot(z2,[t_init,t_fin])
title([])
set(h,'Linewidth',3)
xlabel('Time,s')
ylabel('Parametric Function')
set(gca,'Fontsize',30)
grid on
z1=diff(z2,t)-11*z2;
% subplot(2,2,2)
% 
% h1=ezplot(z1,[t_init,t_fin])
% title([])
% set(h1,'Linewidth',3)
% xlabel('Time,s')
% ylabel('Parametric Function')
% set(gca,'Fontsize',20)
% grid on
X=T*[z1;z2]

 u=-86.5-316.*t+391.5*t.^2-57*t.^3;
 subplot(1,2,2)
 h2=ezplot(u,[t_init,t_fin])
 title([])
 set(h2,'Linewidth',3,'Color',[0.7 0 0])
 xlabel('Time,s')
 ylabel('Controller function')
 set(gca,'Fontsize',30)
 grid on
 figure(2)
 subplot(1,2,1)
 h3=ezplot(X(1),[0 1])
 title([])
 set(h3,'Linewidth',3,'Color',[0 0.3 0])
 
 xlabel('Time,s')
 ylabel('First state trajectory')
 set(gca,'Fontsize',30)
 grid on
 subplot(1,2,2)
 h4=ezplot(X(2),[0 1])
 title([])
 set(h4,'Linewidth',3,'Color',[0 0 0.9])
 
 xlabel('Time,s')
 ylabel('Second state trajectory')
 set(gca,'Fontsize',30)
 grid on
 
% z=-10.5+4.5.*t+13*t.^2-9.5*t.^3;
% x1=9+52.*t-57*t.^2
% x2=6+165.*t-145*t.^2+19*t.^3
% subplot(2,2,1)
% plot(t,x1,'linewidth',3,'Color',[0 0.7 0])
% grid on
% xlabel('time,s')
% ylabel('Evolution of first state')
% set(gca,'fontsize',20)
% subplot(2,2,2)
% plot(t,x2,'linewidth',3,'Color',[0.7 0 0])
% grid on
% xlabel('time,s')
% ylabel('Evolution of second state')
% set(gca,'fontsize',20)
