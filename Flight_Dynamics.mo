package Flight_Dynamics
extends Modelica.Icons.Package;
class Components
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
    Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

//Change in eileron, rudder, and elevator angles

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
Real Cn;

//Yaw coeff
      //// environmental constants
      parameter Real rho = 1.225;       // air desnsity
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
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

//Moments

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

Force[1] = -D*cos(alpha)+L*sin(alpha)+thrust[1] ;
Force[3] = -D*sin(alpha)-L*cos(alpha);
Force[2] = CY*Q*s;



end ForceMoment_Gen;



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
    
Modelica.Blocks.Interfaces.RealOutput v[3] annotation(
    Placement(visible = true, transformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0))); //Linear velocity
Modelica.Blocks.Interfaces.RealOutput pos[3]annotation(
    Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //Position (Displacement) //Displacement
Modelica.Blocks.Interfaces.RealOutput omega[3] annotation(
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //Angular velocity around the CM
Modelica.Blocks.Interfaces.RealOutput angles[3] annotation(
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
  vdot = 1 / mass * Force  +DCM*g + OMEGA * v;
  der(v) = vdot;
  der(pos) = inv(DCM)*v;
  omegadot = inv(J) * (Moment- OMEGA * J * omega);
  der(omega) = omegadot;
  euler_rates = Rotation_mat * omega;
  der(angles) = euler_rates;

annotation(
    uses(Modelica(version = "3.2.2")));
end Flight6DOF;




end Components;
class Test_Cases

model CessnaTrim

// The initial values for delta[2] (elevator), alpha, thrust, and V are obtained by executing Trim_Conditions_Cessna.mo.

parameter Real m = 1043.26;//1.56 for zagi
parameter Real s = 16.1651;//reference area
parameter Real cbar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.81};//gravitational force
//parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;



parameter Real CD0    = 0.036;//= 0.01631;for Zagi
parameter Real K_drag  = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_beta  = -0.31;//for cessna
parameter Real Cy_p  = -0.037;//for cessna
parameter Real Cy_r   = 0.21;//for cessna
parameter Real Cy_delta_r = 0.187; //for cessna
parameter Real Cy_delta_a= 0;     //for cessna

// lift
parameter Real CL0 = 0.25;   //for cessna
parameter Real CL_alpha = 4.47;//for cessna
parameter Real CL_q = 3.9;//for cessna
parameter Real CL_delta_e = 0.3476;//for cessna

// rolling moment
parameter Real Cl_beta = -0.089;//for cessna
parameter Real Cl_p = -0.47;//for cessna
parameter Real Cl_r = 0.096;//for cessna
parameter Real Cl_delta_a= -0.09;//for cessna
parameter Real Cl_delta_r = 0.0147;//for cessna

// pitching moment
parameter Real Cm0 = -0.02;//for cessna
parameter Real Cm_alpha = -1.8;//for cessna
parameter Real Cm_q   = -12.4;//for cessna
parameter Real Cm_delta_e = -1.28;//for cessna

// yawing moment
parameter Real Cn_beta = 0.065;//for cessna
parameter Real Cn_p  = -0.03;//for cessna
parameter Real Cn_r = -0.99;//for cessna
parameter Real Cn_delta_a = -0.0053;//for cessna
parameter Real Cn_delta_r = -0.0657;//for cessna


//Initial conditions. (delta[2], thrust[1] and the others are straightforward)
parameter  Real del[3] = {0,-0.15625,0};
parameter  Real thrust[3] = {1112.82 , 0, 0};
parameter Real alphazero = 0.1;
parameter Real V = 39.8858;

  
  Flight_Dynamics.Components.ForceMoment_Gen forceMoment_Gen1( CD0 = CD0, CD_alpha = CD_alpha, CD_beta = CD_beta, CD_delta_e = CD_delta_e, CD_q = CD_q, CL0 = CL0, CL_alpha = CL_alpha, CL_delta_e = CL_delta_e, CL_q = CL_q, Cl_beta = Cl_beta, Cl_delta_a = Cl_delta_a, Cl_delta_r =Cl_delta_r, Cl_p = Cl_p, Cl_r = Cl_r, Cm0 = Cm0, Cm_alpha = Cm_alpha, Cm_delta_e = Cm_delta_e, Cm_q = Cm_q, Cn_beta = Cn_beta, Cn_delta_a = Cn_delta_a, Cn_delta_r = Cn_delta_r, Cn_p = Cn_p, Cn_r = Cn_r, Cy_beta = Cy_beta, Cy_delta_a = Cy_delta_a, Cy_delta_r = Cy_delta_r, Cy_p = Cy_p, Cy_r = Cy_r,W =m * {0, 0, 9.81},  b= b, cbar =cbar, g = 9.81, m = m, rho = 1.225, s = s)  annotation(
    Placement(visible = true, transformation(origin = {-9, 1}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));



  Flight_Dynamics.Components.Flight6DOF flight6DOF1(J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}}, g = {0,0, 9.81},mass = m, omega( fixed = true,start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {V*cos(alphazero), 0, V*sin(alphazero)}))  annotation(
    Placement(visible = true, transformation(origin = {51, 1}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
  
  



initial equation
forceMoment_Gen1.alpha = alphazero;
forceMoment_Gen1.angles[2] = alphazero;

equation
  connect(flight6DOF1.v, forceMoment_Gen1.vel) annotation(
    Line(points = {{61, 9}, {90, 9}, {90, 46}, {-10, 46}, {-10, 20}, {-9, 20}}, color = {0, 0, 127}, thickness = 0.5));


  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{61, -7}, {102, -7}, {102, 54}, {-70, 54}, {-70, 20}, {-17.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{61, -2}, {98, -2}, {98, 32}, {-0.5, 32}, {-0.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.Moment, forceMoment_Gen1.Moment) annotation(
    Line(points = {{41, -3}, {21.5, -3}, {21.5, -4}, {10, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{10, 6}, {41, 6}}, color = {0, 0, 127}, thickness = 0.5));
  forceMoment_Gen1.thrust = thrust;
  forceMoment_Gen1.delta = del;
  
  annotation(experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-3, Interval = 0.001),
    uses(Modelica(version = "3.2.2")));
    end CessnaTrim;









    
model Trim_Conditions_Cessna


import Modelica.SIunits.*;
import Modelica.Math.Matrices.*;

parameter Real m = 1043.26;//
parameter Real s = 16.1651;//reference area
parameter Real cbar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.81};//gravitational force



parameter Real CD0    = 0.036;//
parameter Real K_drag  = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_beta  = -0.31;//for cessna
parameter Real Cy_p  = -0.037;//for cessna
parameter Real Cy_r   = 0.21;//for cessna
parameter Real Cy_delta_r = 0.187; //for cessna
parameter Real Cy_delta_a= 0;     //for cessna

// lift
parameter Real CL0 = 0.25;   //for cessna
parameter Real CL_alpha = 4.47;//for cessna
parameter Real CL_q = 3.9;//for cessna
parameter Real CL_delta_e = 0.3476;//for cessna

// rolling moment
parameter Real Cl_beta = -0.089;//for cessna
parameter Real Cl_p = -0.47;//for cessna
parameter Real Cl_r = 0.096;//for cessna
parameter Real Cl_delta_a= -0.09;//for cessna
parameter Real Cl_delta_r = 0.0147;//for cessna

// pitching moment
parameter Real Cm0   = -0.02;//for cessna
parameter Real Cm_alpha = -1.8;//for cessna
parameter Real Cm_q  = -12.4;//for cessna
parameter Real Cm_delta_e= -1.28;//for cessna

// yawing moment
parameter Real Cn_beta = 0.065;//for cessna
parameter Real Cn_p  = -0.03;//for cessna
parameter Real Cn_r = -0.99;//for cessna
parameter Real Cn_delta_a = -0.0053;//for cessna
parameter Real Cn_delta_r = -0.0657;//for cessna

parameter Real rho = 1.225;
parameter Real[3,3] J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}};
Real L;
Real D;

Real Q;
  

Real V;

parameter Real[3] omega = {0,0.0,0};

Real CL;
Real CD;
parameter Real alpha = 0.1;
Real de;//To be pasted in delta[2] in the TestFm file
Real thrust;//To be pasted in thrust[1] in the TestFm file
Real theta = alpha;


equation


Q=0.5*rho*V^2;

0  = Cm0+Cm_alpha*alpha+((Cm_q*omega[2]*cbar)/(2*V))+Cm_delta_e*de;
CL = CL0+CL_alpha*alpha+((CL_q*omega[2]*cbar)/(2*V))+CL_delta_e*de;
//CD = CD0+CD_alpha*alpha+((CD_q*omega[2]*cbar)/(2*V))+CD_delta_e*abs(de);// + CDbeta * beta + CDdeltae *
      CD = CD0 + K_drag*CL^2;

//Elevator;
// forces and moments

L = CL*s*Q;
D = CD*s*Q;



0 = -D*cos(alpha)+L*sin(alpha)+thrust - m*9.81*sin(theta);
0 = -D*sin(alpha)-L*cos(alpha)+m*9.81*cos(theta);

end Trim_Conditions_Cessna;
    
model CessnaPerturbed

//The initial values for delta[2] (elevator), alpha, thrust, and V are obtained by executing Trim_Conditions_Cessna.mo.

parameter Real m = 1043.26;//1.56 for zagi
parameter Real s = 16.1651;//reference area
parameter Real cbar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.81};//gravitational force
//parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;



parameter Real CD0    = 0.036;//= 0.01631;for Zagi
parameter Real K_drag  = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_beta  = -0.31;//for cessna
parameter Real Cy_p  = -0.037;//for cessna
parameter Real Cy_r   = 0.21;//for cessna
parameter Real Cy_delta_r = 0.187; //for cessna
parameter Real Cy_delta_a= 0;     //for cessna

// lift
parameter Real CL0 = 0.25;   //for cessna
parameter Real CL_alpha = 4.47;//for cessna
parameter Real CL_q = 3.9;//for cessna
parameter Real CL_delta_e = 0.3476;//for cessna

// rolling moment
parameter Real Cl_beta = -0.089;//for cessna
parameter Real Cl_p = -0.47;//for cessna
parameter Real Cl_r = 0.096;//for cessna
parameter Real Cl_delta_a= -0.09;//for cessna
parameter Real Cl_delta_r = 0.0147;//for cessna

// pitching moment
parameter Real Cm0 = -0.02;//for cessna
parameter Real Cm_alpha = -1.8;//for cessna
parameter Real Cm_q   = -12.4;//for cessna
parameter Real Cm_delta_e = -1.28;//for cessna

// yawing moment
parameter Real Cn_beta = 0.065;//for cessna
parameter Real Cn_p  = -0.03;//for cessna
parameter Real Cn_r = -0.99;//for cessna
parameter Real Cn_delta_a = -0.0053;//for cessna
parameter Real Cn_delta_r = -0.0657;//for cessna


//Initial conditions. (delta[2], thrust[1] and the others are straightforward)
parameter  Real del[3] = {0,-0.15625,0};
parameter  Real thrust[3] = {1112.82 , 0, 0};
parameter Real alphazero = 0.1;
parameter Real V = 39.8858;

  
  Flight_Dynamics.Components.ForceMoment_Gen forceMoment_Gen1( CD0 = CD0, CD_alpha = CD_alpha, CD_beta = CD_beta, CD_delta_e = CD_delta_e, CD_q = CD_q, CL0 = CL0, CL_alpha = CL_alpha, CL_delta_e = CL_delta_e, CL_q = CL_q, Cl_beta = Cl_beta, Cl_delta_a = Cl_delta_a, Cl_delta_r =Cl_delta_r, Cl_p = Cl_p, Cl_r = Cl_r, Cm0 = Cm0, Cm_alpha = Cm_alpha, Cm_delta_e = Cm_delta_e, Cm_q = Cm_q, Cn_beta = Cn_beta, Cn_delta_a = Cn_delta_a, Cn_delta_r = Cn_delta_r, Cn_p = Cn_p, Cn_r = Cn_r, Cy_beta = Cy_beta, Cy_delta_a = Cy_delta_a, Cy_delta_r = Cy_delta_r, Cy_p = Cy_p, Cy_r = Cy_r,W =m * {0, 0, 9.81},  b= b, cbar =cbar, g = 9.81, m = m, rho = 1.225, s = s)  annotation(
    Placement(visible = true, transformation(origin = {-9, 1}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));



  Flight_Dynamics.Components.Flight6DOF flight6DOF1(J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}}, g = {0,0, 9.81},mass = m, omega( fixed = true,start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {V*cos(alphazero), 0, V*sin(alphazero)}))  annotation(
    Placement(visible = true, transformation(origin = {51, 1}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
  
  
Modelica.Blocks.Sources.RealExpression[3] realExpression1(y = if time > 10 and time < 15 then {0, del[2] + 3.1412 /180, 0} else del)  annotation(    Placement(visible = true, transformation(origin = {-112, 1}, extent = {{-26, -47}, {26, 47}}, rotation = 0)));



initial equation
forceMoment_Gen1.alpha = alphazero;
forceMoment_Gen1.angles[2] = alphazero;

equation


  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{61, -7}, {102, -7}, {102, 54}, {-70, 54}, {-70, 20}, {-17.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{61, -2}, {98, -2}, {98, 32}, {-0.5, 32}, {-0.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.v, forceMoment_Gen1.vel) annotation(
    Line(points = {{61, 9}, {90, 9}, {90, 42}, {-58, 42}, {-58, 20}, {-9, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.Moment, forceMoment_Gen1.Moment) annotation(
    Line(points = {{41, -3}, {21.5, -3}, {21.5, -4}, {10, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{10, 6}, {41, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.delta, realExpression1.y) annotation(    Line(points = {{-28, -4}, {-58.5, -4}, {-58.5, 1}, {-83, 1}}, color = {0, 0, 127}, thickness = 0.5));
  forceMoment_Gen1.thrust = thrust;
  
  
 
  
  annotation(experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-3, Interval = 0.001),
    uses(Modelica(version = "3.2.2")));
    end CessnaPerturbed;








    
    



end Test_Cases;
end Flight_Dynamics;