block Flight6DOF

import Modelica.Math.Matrices.*;
import SI=Modelica.SIunits;
import Modelica.Blocks.Interfaces.*;


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



RealInput Force[3]annotation(
    Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Force
RealInput Moment[3]annotation(
    Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Momentum
    
Modelica.Blocks.Interfaces.RealOutput v[3](each start = 0, each fixed = true ) annotation(
    Placement(visible = true, transformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0))); //Linear velocity
Modelica.Blocks.Interfaces.RealOutput pos[3](each start = 0,each fixed = true )annotation(
    Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //Position (Displacement) //Displacement
Modelica.Blocks.Interfaces.RealOutput omega[3](each start = 0, each fixed = true ) annotation(
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //Angular velocity around the CM
Modelica.Blocks.Interfaces.RealOutput angles[3](each start = 0, each fixed = true ) annotation(
    Placement(visible = true, transformation(origin = {110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //Angular displacement
  parameter Real mass = 1;
parameter Real g[3] = {0, 0, 9.8};
parameter Real J[3,3] = mass*{{1, 0, 0},{0,1,0},{0,0,1}};//Moment of Inertia
Real vdot[3];//Linear Acceleration
Real omegadot[3];//Angular acceleration
Real OMEGA[3,3] = skew(omega);//Skew symmetric matrix form of the angular velocity term
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);//The direction cosine matrix
Real Rotation_mat[3,3] = {{1, tan(angles[2])*sin(angles[1]), tan(angles[2])*cos(angles[1])}, {0, cos(angles[1]), -sin(angles[1])},{0, sin(angles[1])/cos(angles[2]) , cos(angles[1])/cos(angles[2])}};
Real euler_rates[3];


equation
  vdot = 1 / mass * Force + DCM * g + OMEGA * v;
  der(v) = vdot;
  der(pos) = inv(DCM)*v;
  omegadot = inv(J) * (Moment- OMEGA * J * omega);
  der(omega) = omegadot;
  euler_rates = Rotation_mat * omega;
  der(angles) = euler_rates;

annotation(
    uses(Modelica(version = "3.2.2")));
end Flight6DOF;