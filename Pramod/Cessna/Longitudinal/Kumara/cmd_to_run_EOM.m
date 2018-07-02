clear all
clc
[t,x]=ode45(@EOM,[0 5],[100 0 0 0 0 0 0 0 100 0 0 0]);
plot(t,x)