model val_gen

import Modelica.SIunits.*;

parameter Real CL0 = 0.25;
parameter Real CLa = 4.47 ;//CL alpha slope
parameter Real CLq  = 1.7;


// drag 
parameter Real CD0 = 0.036;//minimum drag
parameter Real CDCL =  0.3;//CL^2 term for drag polar

// pitch moment
parameter Real Cm0 = -0.02;//Base value for pitch
parameter Real Cma = -1.8 ;//alpha effect on pitch, <0 for stability
parameter Real Cmde = -1.28;//elevator effect on pitch

parameter Real m = 1043.26;
parameter Real g = 9.81;
parameter Real rho = 1.225;
parameter Real V = 65;
parameter Real cBar = 1.493 ;//average chord
parameter Real s = 16.1651;//reference area

Real CL;
Real CD;
Real alpha;
Real de;
Real thrust;
parameter Real CLde = 0.3476;

equation
Cm0 + Cma*alpha + Cmde* de = 0;
CL = CL0 + CLa*alpha +  CLde*de;
CD = CD0 + CDCL*CL^2;
//(0.5*rho*V^2)*s*(CD*cos(alpha) + CL*sin(alpha)) - (m*g)*sin(alpha) + thrust*sin(alpha) = 0;
//(0.5*rho*V^2)*s*(CD*sin(alpha) + CL*cos(alpha)) -thrust (m*g)*sin(alpha) = 0;
(0.5*rho*V^2)*s*CL -m*g + thrust*sin(alpha) = 0;
thrust*cos(alpha) - (0.5*rho*V^2)*s* CD = 0;
end val_gen;