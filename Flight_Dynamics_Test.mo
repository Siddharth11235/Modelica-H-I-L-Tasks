

model Flight_Dynamics_Test

import Modelica.Math.Matrices.*;
import SI=Modelica.SIunits;
function T1
  
  input Real a;
  output Real T[3,3];
algorithm
  T := {{  1,      0,      0}, {  0, cos(a), sin(a)},{  0,-sin(a), cos(a)}};
end T1;

function T2
  input Real a;
  output Real T[3,3];
algorithm
  T := {{ cos(a),  0,-sin(a)}, {  0,      1,      0},{ sin(a),  0, cos(a)}};
end T2;

function T3
  input Real a;
  output Real T[3,3];
algorithm
  T := {{ cos(a), sin(a), 0},{-sin(a), cos(a), 0},{      0,      0, 1}};
end T3;

Real Force[3] = {1,0,0};
Real Moment[3] = {0,0,1};
parameter Real mass = 1;
parameter Real g[3] = {0, 0, -9.8};
parameter Real J[3,3] = mass*{{1, 0, 0},{0,1,0},{0,0,1}};
Real acc[3];
Real vel[3];
Real pos[3];
Real ang_acc[3];
Real ang_vel[3];
Real angles[3];
Real Omega[3,3] = skew(ang_vel);
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);

equation
acc = (1/mass)*Force + DCM*g + Omega*vel;
der(vel) = acc;
der(pos) = inv(DCM)*vel;
ang_acc = inv(J)*(Moment - Omega*J*vel);
der(ang_vel) = ang_acc;
der(angles) = ang_vel;
end Flight_Dynamics_Test;