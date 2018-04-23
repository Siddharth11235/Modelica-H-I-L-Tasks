block ForceMoment_Gen

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




RealInput alpha annotation(
    Placement(visible = true, transformation(origin = {-110, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Angle of Attack
RealInput beta annotation(
    Placement(visible = true, transformation(origin = {-110, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Side slip angle


RealInput[3] Thrust annotation(
    Placement(visible = true, transformation(origin = {-110, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force
    
RealInput[3] delta annotation(
    Placement(visible = true, transformation(origin = {-110, -90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -90}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Change in eileron, rudder, and elevator angles   

RealOutput Force[3]annotation(
    Placement(visible = true, transformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Force
RealOutput Moment[3]annotation(
    Placement(visible = true, transformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Momentum

parameter Real s;//reference area
parameter Real cBar ;//average chord
parameter Real b ;//span
parameter Real Fg[3]  = {0,0, -9.8};//gravitational force
Real qBar;//Pressure
Real CL; //Coeff of Lift
Real CD;//Coeff of Drag
Real CY;//Coeff of Sideslip
Real Cl;//Roll coeff
Real Cm;//Pitch coeff
Real Cn;//Yaw coeff 

// lift
parameter Real CL0;
parameter Real CLa ;//CL alpha slope


// drag 
parameter Real CD0;//minimum drag
parameter Real CDCL;//CL^2 term for drag polar

// side force
Real CYb;//side slipe effect on side force
Real CYda;//Aileron effects on sideslip coeff
Real CYdr;//rudder effects on sideslip coeff

// roll moment
Real Cldr;//rudder effects on roll
Real Clda;//Aileron effect on roll

// pitch moment
Real Cm0 ;//Base value for pitch
Real Cma ;//alpha effect on pitch, <0 for stability
Real Cmde;//elevator effect on pitch
Real Cnde;//elevator effects on yaw
Real Cndr;//rudder effects on yaw

Real angles[3](each start = 0,each fixed = true );//Angular displacments
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);//The direction cosine matrix

Real Cb_w[3,3] = inv({{cos(alpha)*cos(beta), sin(beta), sin(alpha)*cos(beta)},{-cos(alpha)*sin(beta), cos(beta), -sin(alpha)*sin(beta)},{-sin(alpha), 0, cos(alpha)}});

equation
  CL = CL0 + CLa*alpha;
  CD = CD0 + CDCL*CL^2;
  CY = CYb*beta + CYda*delta[1] + CYdr*delta[2] ;
  Cl = Cldr*delta[2] + Clda*delta[1];
  Cm = Cma*alpha + Cm0 + Cmde*delta[2];
  Cn = Cndr*delta[2] + Cndr*delta[3];
  Force = Cb_w*{{-CD,-CY,-CL}* qBar * s + DCM*Fg + Thrust};//Multiply Fg with DCM and multiply the entire w body to wind (2.3.2)
  Moment = {Cl*b,Cm*cBar,Cn*b}*qBar*s;
end ForceMoment_Gen;

