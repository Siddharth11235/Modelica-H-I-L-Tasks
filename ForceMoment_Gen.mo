block ForceMoment_Gen

import Modelica.Math.Matrices.*;
import Modelica.SIunits.*;
import Modelica.Blocks.Interfaces.*;
import Modelica.Math.Vectors.*;

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







RealInput[3] Thrust annotation(
    Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force
    
RealInput[3] delta annotation(
    Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Change in eileron, rudder, and elevator angles   



RealInput[3] angles annotation(
    Placement(visible = true, transformation(origin = {-50, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-50, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));//Angular Displacement
    
RealInput[3] vel annotation(
    Placement(visible = true, transformation(origin = {0, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));//Velocity

RealInput[3] omega annotation(
    Placement(visible = true, transformation(origin = {50, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {50, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));//Euler velocity


RealInput alpha annotation(
    Placement(visible = true, transformation(origin = {-110, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force
    
    
    
parameter Real m = 1043.26;
parameter Real s = 16.1651;//reference area
parameter Real cBar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, -9.8};//gravitational force
Real CL; //Coeff of Lift
Real CD;//Coeff of Drag
Real CY;//Coeff of Sideslip
Real Cl;//Roll coeff
Real Cm;//Pitch coeff
Real Cn;//Yaw coeff 

//Angle of sideslip
Angle beta(start = 0.0);

// lift
parameter Real CL0 = 0.25;
parameter Real CLa = 4.47 ;//CL alpha slope
parameter Real CLq  = 1.7;
parameter Real CLde = 0.3476;

// drag 
parameter Real CD0 = 0.036;//minimum drag
parameter Real CDCL =  0.3;//CL^2 term for drag polar

// side force
Real CYb  = -0.31;//side slipe effect on side force
Real CYda =  0;//Aileron effects on sideslip coeff
Real CYdr = 0.21;//rudder effects on sideslip coeff

// roll moment
Real Cldr= 0.0147;//rudder effects on roll
Real Clda =-0.09;//Aileron effect on roll

// pitch moment
Real Cm0 = -0.02;//Base value for pitch
Real Cma = -1.8 ;//alpha effect on pitch, <0 for stability
Real Cmde = -1.28;//elevator effect on pitch

//Yawing Moment
Real Cnda  = -0.0053;//Aileron effects on yaw
Real Cndr = -0.0657;//rudder effects on yaw
Real Cnb = 0.065;//Sideslip effects on yaw
Real Cnp = -0.03;//pitching effect on yaw
Real Cnr = -0.99;//rolling effect on yaw
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);//The direction cosine matrix

Real Cw_b[3,3] = {{cos(alpha)*cos(beta), sin(beta), sin(alpha)*cos(beta)},{-cos(alpha)*sin(beta), cos(beta), -sin(alpha)*sin(beta)},{-sin(alpha), 0, cos(alpha)}};

Real vw[3] = {0,0,0};
Real vrel[3] = vel - vw;
Real qBar = 0.5*1.225*(norm(vrel))^2;//Pressure


RealOutput Force[3] (start = {1043.26 * 9.8 / 8, 0, 1043.26 * 9.8})annotation(
    Placement(visible = true, transformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Force
RealOutput Moment[3] (start = {0,0,0})annotation(
    Placement(visible = true, transformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Momentum
    
equation
 // alpha= atan(vrel[1]/vel[1]);
  beta  = asin(vrel[2]/norm(vrel));
  CL = CL0 + CLa*alpha + CLq*omega[2]*cBar/(2*norm(vrel)) + CLde*delta[2];
  CD = CD0 + CDCL*CL^2;
  CY = CYb*beta + CYda*delta[1] + CYdr*delta[3] ;
  Cl = Cldr*delta[3] + Clda*delta[1];
  Cm = Cma*alpha + Cm0 + Cmde*delta[2];
  Cn = Cndr*delta[3] + Cndr*delta[3] + Cnb*beta + Cnp*omega[1] + Cnr*omega[3];
  Force = Cw_b*({-CD,-CY,-CL}*qBar*s) + DCM*W + Thrust;//Multiply Fg with DCM and multiply the entire w body to wind (2.3.2)
  Moment = {Cl*b,Cm*cBar,Cn*b}*qBar*s;
end ForceMoment_Gen;

