model Flight_Dynamics_Test
  import Modelica.Math.Matrices.*;
  import Modelica.SIunits.*;
  import Modelica.Blocks.Interfaces.*;
  import Modelica.Math.Vectors.*;

  function T1
    input Real a;
    output Real T[3, 3];
  algorithm
    T := {{1, 0, 0}, {0, cos(a), sin(a)}, {0, -sin(a), cos(a)}};
  end T1;

  function T2
    input Real a;
    output Real T[3, 3];
  algorithm
    T := {{cos(a), 0, -sin(a)}, {0, 1, 0}, {sin(a), 0, cos(a)}};
  end T2;

  function T3
    input Real a;
    output Real T[3, 3];
  algorithm
    T := {{cos(a), sin(a), 0}, {-sin(a), cos(a), 0}, {0, 0, 1}};
  end T3;

  parameter Real delta[3] = {0, -0.0304977268414434, 0};
  parameter Real thrust[3] = {1424.12, 0, 0};
  parameter Real m = 1043.26;
  parameter Real s = 16.1651;
  //reference area
  parameter Real cbar = 1.493;
  //average chord
  parameter Real b = 10.911;
  //span
  parameter Real W[3] = m * {0, 0, -9.8};
  Real[3] vel  ={ 60*cos(0.010007), 0, 60*sin(0.010007)};
  Real[3] omega = {0,0,0.0};
  Real[3] angles = {0,0,0.0};
  Real[3] pos (start = {0,0,-1000});
  Real OMEGA[3,3] = skew(omega);//Skew symmetric matrix form of the angular velocity term

  Real[3] vdot;
  ForceMoment_Gen forceMoment_Gen1 annotation(
    Placement(visible = true, transformation(origin = {0, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Real DCM[3,3] = T1(angles[1])*T2(angles[2])*T3(angles[3]);//The direction cosine matrix




  equation
   
     
  forceMoment_Gen1.thrust = thrust;
  forceMoment_Gen1.delta = delta;
  forceMoment_Gen1.vel = vel;
  forceMoment_Gen1.angles = angles;
  forceMoment_Gen1.omega = omega;
  der(pos) = vel;
 vdot = 1 / m * forceMoment_Gen1.Force + DCM *{0,0, 9.81} + OMEGA * vel;
 
end Flight_Dynamics_Test;