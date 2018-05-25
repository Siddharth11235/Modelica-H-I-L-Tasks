

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

Real Force[3] = {0,0,9.8};
Real Moment[3] = {0,0,1};
parameter Real mass = 1;
parameter Real g[3] = {0, 0, -9.8};
parameter Real J[3,3] = mass*{{1, 0, 0},{0,1,0},{0,0,1}};//Moment of Inertia
Real vdot[3];//Linear Acceleration
Real v[3](each start = 0, each fixed = true );//Velocity
Real pos[3](each start = 0,each fixed = true );//Position (Displacement)
Real omegadot[3];//Angular acceleration
Real omega[3](start = {1.0,0,0},each fixed = true );//Angular velocity around the CM
Real angles[3](each start = 0,each fixed = true );//Angular displacments
Real OMEGA[3,3] = -skew(omega);//Skew symmetric matrix form of the angular velocity term
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);//The direction cosine matrix
Real Rotation_mat[3,3] = {{1, tan(angles[2])*sin(angles[1]), tan(angles[2])*cos(angles[1])}, {0, cos(angles[1]), -sin(angles[1])},{0, sin(angles[1])/cos(angles[2]) , cos(angles[1])/cos(angles[2])}};
Real euler_rates[3];

equation
vdot = (1/mass)*Force + DCM*g + OMEGA*v;
der(v) = vdot;
der(pos) = inv(DCM)*v;
omegadot = inv(J)*(Moment - OMEGA*J*omega);
der(omega) = omegadot;
euler_rates = Rotation_mat*omega;
der(angles) = euler_rates;
end Flight_Dynamics_Test;