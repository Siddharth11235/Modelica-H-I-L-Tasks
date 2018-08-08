model linear_WindTrim


import Modelica.Blocks.Interfaces.*;

  parameter Integer n = 4; // states
  parameter Integer k = 2; // top-level inputs
  parameter Integer l = 2; // top-level outputs
  parameter Real x0[4] = {39.8858212875252,0.09999999999999316,2.18650820700695e-07,-9.989611474825446e-15};
  parameter Real u0[2] = {-0.15625,1112.82};
  
  
  
  
  parameter Real A[12,12] = [-0.05321913896678525,-7.309325638062395,-0,-0,-9.809999999999766,-0,-0,-0.1176174797773325,-0,-0,-0,-0;-0.01219893117742086,-1.718678333264892,0,0,-5.377762026288438e-08,0,0,0.972369652215591,0,0,0,0;0,0,-0.1173471007801458,0,0,0.2459520622449354,0.0979177134828252,0,-0.9841312554283186,0,0,0;0,0,0,0,0,-9.989611474825684e-15,0,0,1.000000000000024,0,0,0;0,0,0,0,0,0,0,1,0,0,0,0;0,0,0,0,0,-2.184236747451698e-21,1,0,2.186508207006984e-07,0,0,0;0,0,-11.9005831358691,0,0,0,-8.595915052137459,0,1.755761372351488,0,0,0;8.72055317654725e-15,-23.19570991744142,0,0,0,0,0,-2.990667436458215,0,0,0,0;0,0,4.188841114696014,0,0,0,-0.2644343119382115,0,-8.726332293961047,0,0,0;0.9999999999999761,0,0,0,-8.721067558838564e-06,0,0,0,0,0,0,0;0,0,0,39.88582128752424,0,0,0,0,0,0,0,0;-2.186508207006932e-07,-0,-0,-0,-39.88582128752424,-0,-0,-0,-0,-0,-0,-0];
  
   parameter Real A_lon[4,4] = {{A[1,1],A[1,2], A[1,5],A[1,8]},{A[2,1],A[2,2], A[2,5],A[2,8]},{A[5,1],A[5,2], A[5,5],A[5,8]},{A[8,1],A[8,2], A[8,5],A[8,8]}};
 
  
  parameter Real B[12,2] = [-0.5601131348246953,0.0009537451500853349;-0.1315801081873247,-2.399191085901589e-06;0,0;0,0;0,0;0,0;0,0;-16.49472705240279,-0;0,0;0,0;0,0;0,0];
  
  parameter Real B_lon[4,2] = [B[1,:],B[2,:];B[5,:],B[8,:]];
  
  parameter Real C[2,12] = [-0.01219893117742086,-1.718678333264892,0,0,-5.377762026288438e-08,0,0,0.972369652215591,0,0,0,0;8.72055317654725e-15,-23.19570991744142,0,0,0,0,0,-2.990667436458215,0,0,0,0];
  
  parameter Real C_lon[2,4] = {{C[1,1],C[1,2],C[1,5],C[1,8]},{C[2,1],C[2,2],C[2,5],C[2,8]}};
  
  
  
  parameter Real D_lon[2,2] = [-0.1315801081873247,-2.399191085901589e-06;-16.49472705240279,-0];
  
  
  
  Real x[4](start=x0);
RealInput deltaE (start = -0.15625) annotation(start = 0.1,Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

RealInput thrust (start = 1112.82) annotation(Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force = 1112.82;

Modelica.Blocks.Interfaces.RealOutput q  (start = 0) annotation(Placement(visible = true, transformation(origin = {110, 33}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //Angular velocity 


Modelica.Blocks.Interfaces.RealOutput alpha (start = 0.1) annotation( Placement(visible = true, transformation(origin = {110, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //Angle of attack 


/*
  Real 'x_V' = x[1];
Real 'x_alpha' = x[2];
Real 'x_beta' = x[3];
Real 'x_chi' = x[4];
Real 'x_gamma' = x[5];
Real 'x_mu' = x[6];
Real 'x_p' = x[7];
Real 'x_q' = x[8];
Real 'x_r' = x[9];
Real 'x_x' = x[10];
Real 'x_y' = x[11];
Real 'x_z' = x[12];
Real 'u_deltaE' = u[1];
Real 'u_thrust' = u[2];
Real 'y_alpha' = y[1];
Real 'y_q' = y[2];
*/
equation
  der(x) = A_lon * x + B_lon * {deltaE,thrust};
  {alpha,q} = C_lon * x + D_lon * {deltaE,thrust};
end linear_WindTrim;
