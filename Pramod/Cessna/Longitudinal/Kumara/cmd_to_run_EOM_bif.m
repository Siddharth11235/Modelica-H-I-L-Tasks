clear all
clc
U0=[0.3; 0; 0; 0; 0; 0; 0; 0;]
U=fsolve(@EOM_bif,U0)