block AeroCoeffs
import Modelica.Math.Matrices.*;
import Modelica.SIunits.*;
import Modelica.Blocks.Interfaces.*;
import Modelica.Math.Vectors.*;




RealInput[3] vel annotation(
    Placement(visible = true, transformation(origin = {-110, 66}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));//Change in eileron, rudder, and elevator angles   


RealInput[3] delta annotation(
    Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));//Change in eileron, rudder, and elevator angles   


RealInput[3] omega annotation(
    Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110,0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));//Euler velocity


RealInput alpha annotation(
    Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));//Euler velocity


RealInput beta annotation(
    Placement(visible = true, transformation(origin = {-110, -66}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));//Euler velocity    
    
    
    
parameter Real cbar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span


RealOutput[3] ForceCoeffs annotation(Placement(visible = true, transformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Coeff of Drag, Sideslip, Lift


RealOutput[3] MomentCoeffs annotation(Placement(visible = true, transformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Coeff of roll, pitch, yaw




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
  parameter Real Cmalphadot(unit = "/rad/sec") = -12.4 "Cm_alphadot";
  parameter Real Cmdeltae(unit = "/rad") = -1.28 "Cmdelta_e";
  
  parameter Real Cnbeta(unit = "/rad") = 0.065 "Cn_beta";
  parameter Real Cnp(unit = "/rad/s") = -0.03 "Cnp";
  parameter Real Cnr(unit = "/rad/s") = -0.99 "Cnr";
  parameter Real Cndeltaa(unit = "/rad") = -0.0053 "Cndelta_a";
  parameter Real Cndeltar(unit = "/rad") = -0.0657 "Cndelta_r";
  
  parameter Real alphadot = 0.0;



parameter Real vw[3] = {0,0,0};
Real vrel[3] = vel - vw;
  
  equation
  
  
  
  ForceCoeffs[3] = CL0 + CLalpha * alpha + CLq * (omega[2] * cbar) / (2 * norm(vrel)) + CLdeltae * delta[2];
  ForceCoeffs[1] = CD0 + 0.0830304 * ForceCoeffs[3] * ForceCoeffs[3];
// + CDbeta * beta + CDdeltae * Elevator;
  ForceCoeffs[2] = CYb * beta + CYp * (omega[1] * b) / (2 * norm(vrel)) + CYr * (omega[3] * b) / (2 * norm(vrel)) + CYda * delta[1];
  MomentCoeffs[1] = Clbeta * beta + Clp * (omega[1] * b) / (2 * norm(vrel)) + Clr * (omega[3] * b) / (2 * norm(vrel)) + Cldeltaa * delta[1] + Cldeltaa * delta[3];
  MomentCoeffs[2] = Cm0 + Cmalpha * alpha + Cmalphadot * (alphadot * cbar) / (2 * norm(vrel)) + Cmdeltae * delta[2];
  MomentCoeffs[3] = Cnbeta * beta + Cnp * (omega[1] * b) / (2 * norm(vrel)) + Cnr * (omega[3] * b) / (2 * norm(vrel)) + Cndeltaa * delta[1] + Cndeltaa * delta[3];
  
  
  
  
     

end AeroCoeffs;