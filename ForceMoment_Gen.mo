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







RealInput[3] thrust annotation(
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
parameter Real W[3]  = m*{0,0, -9.81};//gravitational force
Real CL; //Coeff of Lift
Real CD;//Coeff of Drag
Real CY;//Coeff of Sideslip
Real Cl;//Roll coeff
Real Cm;//Pitch coeff
Real Cn;//Yaw coeff 


//// environmental constants
parameter Real rho = 1.225; // air desnsity 
parameter Real g   = 9.81; // gravity






//weight

//// aerodynamic coefficients
// drag
parameter Real CD0     = 0.036;
parameter Real K_drag  = 0.0830304;
parameter Real CD_beta = 0.17;
parameter Real CD_alpha;
parameter Real CD_q;
parameter Real CD_delta_e;

//side force
parameter Real Cy_beta    = -0.31;
parameter Real Cy_p       = -0.037;
parameter Real Cy_r       = 0.21;
parameter Real Cy_delta_r = 0.187; 
parameter Real Cy_delta_a = 0;     

// lift
parameter Real CL0;   
parameter Real CL_alpha;
parameter Real CL_q ;
parameter Real CL_delta_e;

// rolling moment
parameter Real Cl_beta;
parameter Real Cl_p ;
parameter Real Cl_r;
parameter Real Cl_delta_a;
parameter Real Cl_delta_r;

// pitching moment
parameter Real Cm0;
parameter Real Cm_alpha ;
parameter Real Cm_q ;
parameter Real Cm_delta_e ;

// yawing moment
parameter Real Cn_beta ;
parameter Real Cn_p ;
parameter Real Cn_r;
parameter Real Cn_delta_a ;
parameter Real Cn_delta_r;

Real L;
Real D;

Real Q;
Real alpha;
Real alphadot;
Real beta;

RealOutput[3] Force annotation(
    Placement(visible = true, transformation(origin = {110, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Forces

RealOutput[3] Moment annotation(
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Moments




equation
alpha = atan2(vel[3],vel[1]);
alphadot = der(alpha);
beta = asin(vel[2] / norm(vel));

Q=0.5*rho*norm(vel)*norm(vel);



CL = CL0+CL_alpha*alpha+((CL_q*omega[2]*cbar)/(2*norm(vel)))+CL_delta_e*delta[2];
//CD =  CD0+CD_alpha*alpha+((CD_q*omega[2]*cbar)/(2*norm(vel)))+CD_delta_e*abs(delta[2]);// + CDbeta * beta + CDdeltae * Elevator;
CD = CD0 + K_drag*CL^2;
CY = Cy_beta * beta + Cy_p * (omega[1] * b) / (2 * norm(vel)) + Cy_r * (omega[3] * b) / (2 * norm(vel)) + Cy_delta_a * delta[1] + Cy_delta_r*delta[3];

Cl = Cl_beta * beta + Cl_p * (omega[1] * b) / (2 * norm(vel)) + Cl_r * (omega[3] * b) / (2 * norm(vel)) + Cl_delta_a * delta[1] + Cl_delta_r * delta[3];
Cm  = Cm0+Cm_alpha*alpha+((Cm_q*omega[2]*cbar)/(2*norm(vel)))+Cm_delta_e*delta[2];
Cn = Cn_beta * beta + Cn_p * (omega[1] * b) / (2 * norm(vel)) + Cn_r * (omega[3] * b) / (2 * norm(vel)) + Cn_delta_a * delta[1] + Cn_delta_r * delta[3];



L = CL*s*Q;
D = CD*s*Q;

Moment[2] = Cm*s*cbar*Q;
Moment[1] = Cl*Q*s*b;
Moment[3] = Cn*Q*s*b;

Force[1] = -D*cos(alpha)+L*sin(alpha)+thrust[1] - m*g*sin(angles[2]);
Force[3] = -D*sin(alpha)-L*cos(alpha)+m*g*cos(angles[2]);
Force[2] = CY*Q*s;





end ForceMoment_Gen;

