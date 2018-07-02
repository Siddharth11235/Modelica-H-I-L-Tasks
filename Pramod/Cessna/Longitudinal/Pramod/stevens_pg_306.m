clc;
clear all;
t=linspace(0,50,1000)
xe=250*cos(-2.5*pi/180)*t;
ze=1500+250*sin(-2.5*pi/180)*t
figure(4)
plot(xe,ze)
