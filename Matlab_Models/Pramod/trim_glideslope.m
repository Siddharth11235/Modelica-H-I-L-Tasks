clear all;
clc
fun = @root2d;
x0 = [0,0];
x = fsolve(fun,x0)