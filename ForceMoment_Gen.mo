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

    
    
    
parameter Real m = 1043.26;
parameter Real s = 16.1651;//reference area
parameter Real cbar = 1.493 ;//average chord
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

Real alpha;//-0.122882;
// lift
 
  
  parameter DimensionlessRatio CL0 = 0.25 "CL0";
  parameter Real CLalpha(unit = "/rad") = 4.7 "CLalpha";
  parameter Real CLq(unit = "/rad/s") = 1.7 "CLq";
  parameter Real CLdeltae(unit = "/rad") = 0.3476 "CLdelta_e";
  parameter DimensionlessRatio CD0 = 0.036 "CD0";
  parameter Real CDbeta(unit = "/rad") = 0.17 "CDbeta";
  parameter Real CDdeltae(unit = "/rad") = 0.06 "CDdelta_e";
  
  parameter Real k =  0.0830304;
  
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
  parameter Real Cmq(unit = "/rad/sec") = -12.4 "Cm_alphadot";
  parameter Real Cmdeltae(unit = "/rad") = -1.28 "Cmdelta_e";
  
  parameter Real Cnbeta(unit = "/rad") = 0.065 "Cn_beta";
  parameter Real Cnp(unit = "/rad/s") = -0.03 "Cnp";
  parameter Real Cnr(unit = "/rad/s") = -0.99 "Cnr";
  parameter Real Cndeltaa(unit = "/rad") = -0.0053 "Cndelta_a";
  parameter Real Cndeltar(unit = "/rad") = -0.0657 "Cndelta_r";
  
  
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);//The direction cosine matrix

Real Cw_b[3,3] = {{cos(alpha)*cos(beta), sin(beta), sin(alpha)*cos(beta)},{-cos(alpha)*sin(beta), cos(beta), -sin(alpha)*sin(beta)},{-sin(alpha), 0, cos(alpha)}};

Real vw[3] = {0,0,0};
Real vrel[3] = vel - vw;
Real qBar = 0.5*1.225*(norm(vrel))^2;//Pressure

Real L;
Real D;
Real Y;



RealOutput Force[3] (start = {0,0,0})annotation(Placement(visible = true, transformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Force
RealOutput Moment[3] (start = {0,0,0})annotation(Placement(visible = true, transformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Momentum
    
    
equation
  alpha= atan2(vrel[3],vel[1]);
  beta  = asin(vrel[2]/norm(vrel));
  CL = CL0 + CLalpha * alpha + CLq * (omega[2] * cbar) / (2 * norm(vrel)) + CLdeltae * delta[2];
  CD = CD0 + 0.0830304 * CL * CL;
// + CDbeta * beta + CDdeltae * Elevator;
  CY = CYb * beta + CYp * (omega[1] * b) / (2 * norm(vrel)) + CYr * (omega[3] * b) / (2 * norm(vrel)) + CYda * delta[1];
  Cl = Clbeta * beta + Clp * (omega[1] * b) / (2 * norm(vrel)) + Clr * (omega[3] * b) / (2 * norm(vrel)) + Cldeltaa * delta[1] + Cldeltaa * delta[3];
  Cm =Cm0+Cmalpha*alpha+((Cmq*omega[2]*cbar)/(2*norm(vrel)))+Cmdeltae*delta[2] ;
  Cn = Cnbeta * beta + Cnp * (omega[1] * b) / (2 * norm(vrel)) + Cnr * (omega[3] * b) / (2 * norm(vrel)) + Cndeltaa * delta[1] + Cndeltaa * delta[3];
  {D,Y,L} = Cw_b*({-CD,-CY,-CL}*qBar*s );//Multiply Fg with DCM and multiply the entire w body to wind (2.3.2)

Force[1]= -D*cos(alpha)+L*sin(alpha)-W[3]*sin(angles[2])+Thrust[1];
Force[2] = Y;
Force[3]= -D*sin(alpha)-L*cos(alpha)+W[3]*cos(angles[2]);

  
  
 Moment = {Cl*b,Cm*cbar,Cn*b}*qBar*s;
end ForceMoment_Gen;

