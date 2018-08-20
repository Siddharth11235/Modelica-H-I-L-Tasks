model linear_FlightLongs

import Modelica.Blocks.Interfaces.*;

  parameter Integer n = 6; // states
  parameter Integer k = 1; // top-level inputs
  parameter Integer l = 1; // top-level outputs
  parameter Real x0[6] = {39.88582128752544,0.1000000000000002,2.186509259551072e-07,-7.378182456164178e-15,19942.9106616831,99.99572989386823};
  parameter Real u0[1] = {-0.15625};
  parameter Real A[6,6] = [-0.0532191411313782,-7.309327515663002,-9.809999999999691,0,0,0;-0.0121989308080552,-1.71867833344851,-6.247500391439244e-08,1.000000000104694,0,0;0,0,0,1,0,0;-1.972829549283974e-16,-23.1957099158693,0,0,0,0;1.000000022551516,0,-1.012928063465349e-05,0,0,0;-2.186509161659518e-07,0,-39.88582128752417,0,0,0];
  parameter Real B[6,1] = [-0.560113147567141;-0.1315801082488951;0;-16.4947270526446;0;0];
  parameter Real C[1,6] = [0,0,0,1,0,0];
  parameter Real D[1,1] = [0];
  Real x[6](start=x0);
  RealInput u[1] (start = -0.15625) annotation( Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  
   Modelica.Blocks.Interfaces.RealOutput y[1] (start = 0) annotation(Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Real 'x_V' = x[1];
Real 'x_alpha' = x[2];
Real 'x_gamma' = x[3];
Real 'x_q' = x[4];
Real 'x_x' = x[5];
Real 'x_z' = x[6];
Real 'u_del' = u[1];
Real 'y_q' = y[1];
equation
  der(x) = A * x + B * u;
  y = C * x + D * u;
end linear_FlightLongs;