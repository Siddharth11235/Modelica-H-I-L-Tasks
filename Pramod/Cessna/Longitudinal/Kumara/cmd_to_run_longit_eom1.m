clear all
clc;
% x0=[1; 1; 1.7; 2; 1; 2.5;];
x0=1.0e+004*[0.0002; -4.2274; -4.2271; 0.0003; -4.2274; -4.2271;];
% x=fsolve(@unicycle2,x0)
options=odeset('RelTol',1e-12,'AbsTol',1e-12);
[t,x] = ode45(@unicycle2,[0 50],x0,options);