
model val_gen

import Modelica.SIunits.*;

// lift

  parameter DimensionlessRatio CL0 = 0.25 "CL0";
  parameter Real CLalpha(unit = "/rad") = 4.7 "CLalpha";
  parameter Real CLq(unit = "/rad/s") = 1.7 "CLq";
  parameter Real CLdeltae(unit = "/rad") = 0.3476 "CLdelta_e";
  
  parameter DimensionlessRatio CD0 = 0.036 "CD0";
  parameter Real CDbeta(unit = "/rad") = 0.17 "CDbeta";
  parameter Real CDdeltae(unit = "/rad") = 0.06 "CDdelta_e";
  
  parameter Real CYb (unit = "/rad") = -0.31 "CYbeta";
  parameter Real CYp(unit = "/rad/s") = -0.037 "CYp";
  parameter Real CYr(unit = "/rad/s") = 0.21 "CYr";
  parameter Real CYda(unit = "/rad") = 0.0 "CYdelta_a";
  
  parameter Real Clbeta(unit = "/rad") = -0.089 "Clbeta";
  parameter Real Clp(unit = "/rad/s") = -0.47 "Clp";
  parameter Real Clr(unit = "/rad/s") = 0.096 "Clr";
  parameter Real Cldeltaa(unit = "/rad") = -0.09 "Cldalta_a";
  parameter Real Cldeltar(unit = "/rad") = 0.0147 "Cldelta_r";
  
  parameter DimensionlessRatio Cm0 = -0.02 "Cm0";
  parameter Real Cmalpha(unit = "/rad") = -1.8 "Cm_alpha";
  parameter Real Cmq(unit = "/rad/sec") = -12.4 "Cm_q";
  parameter Real Cmdeltae(unit = "/rad") = -1.28 "Cmdelta_e";
  
  parameter Real Cnbeta(unit = "/rad") = 0.065 "Cn_beta";
  parameter Real Cnp(unit = "/rad/s") = -0.03 "Cnp";
  parameter Real Cnr(unit = "/rad/s") = -0.99 "Cnr";
  parameter Real Cndeltaa(unit = "/rad") = -0.0053 "Cndelta_a";
  parameter Real Cndeltar(unit = "/rad") = -0.0657 "Cndelta_r";
  

parameter Real m = 1043.26;
parameter Real g = 9.81;
parameter Real rho = 1.225;
parameter Real V = 60;
parameter Real cbar = 1.493 ;//average chord
parameter Real s = 16.1651;//reference area

parameter Real[3] omega = {0,0.0,0};
parameter Real theta = 0;

Real CL;
Real CD;
Real alpha;
Real de;
Real thrust;


Real q;
Real L;
Real D;


equation

q=0.5*rho*V^2;

CL = CL0+CLalpha*alpha+((CLq*omega[2]*cbar)/(2*V))+CLdeltae*de;
0  = Cm0+Cmalpha*alpha+((Cmq*omega[2]*cbar)/(2*V))+Cmdeltae*de ;
CD = CD0+0.0830304*CL^2;

// forces and moments

L = CL*s*q;
D = CD*s*q;



0 = -D*cos(alpha)+L*sin(alpha)-m*g*sin(theta)+thrust;
0 = -D*sin(alpha)-L*cos(alpha)+m*g*cos(theta);


//(0.5*rho*V^2)*s*CL -m*g = 0;
//thrust*cos(alpha) - (0.5*rho*V^2)*s*CD = 0;
end val_gen;