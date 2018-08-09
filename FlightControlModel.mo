model FlightControlModel

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

Modelica.Blocks.Interfaces.RealOutput q  (start = 0) annotation(Placement(visible = true, transformation(origin = {110, 33}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  
//Angular velocity

Modelica.Blocks.Interfaces.RealOutput alpha (start = 0.1) annotation( Placement(visible = true, transformation(origin = {110, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  
//Angle of attack
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

  
  Modelica.Blocks.Sources.Constant const(k = 1112.82)  annotation(
    Placement(visible = true, transformation(origin = {39, 67}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
  parameter Real[2,4] k_place = [77.19313887883463,	6075.030661023554,	122.31655064066875,	-6717.298968943441;
-0.2891312664172394,	-22.733145450996215,	-0.5318537635280807,	25.07093397939501];
  Modelica.Blocks.Math.Add add1 annotation(
    Placement(visible = true, transformation(origin = {-36, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {-16, -84}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {-18, -58}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2 annotation(
    Placement(visible = true, transformation(origin = {-68, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Continuous.PI PI(T = 1)  annotation(
    Placement(visible = true, transformation(origin = {4, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 3}, b = {1, 5})  annotation(
    Placement(visible = true, transformation(origin = {34, -84}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferFunction2(a = {0.6, 1}, b = {1})  annotation(
    Placement(visible = true, transformation(origin = {30, -56}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add3 annotation(
    Placement(visible = true, transformation(origin = {42, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain3(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {2, 44}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  linear_WindTrim linear_WindTrim1 annotation(
    Placement(visible = true, transformation(origin = {80, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse pulse1(amplitude = 0.15625, offset = -0.15625, period = 50, width = 25)  annotation(
    Placement(visible = true, transformation(origin = {-96, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add4 annotation(
    Placement(visible = true, transformation(origin = {-63, 1}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
algorithm

  equation
  connect(linear_WindTrim1.alpha, add4.u2) annotation(
    Line(points = {{92, 2}, {92, 2}, {92, -24}, {-76, -24}, {-76, -2}, {-72, -2}, {-72, -4}}, color = {0, 0, 127}));
  connect(pulse1.y, add4.u1) annotation(
    Line(points = {{-84, 4}, {-72, 4}, {-72, 6}, {-72, 6}}, color = {0, 0, 127}));
  connect(add4.y, add1.u1) annotation(
    Line(points = {{-56, 0}, {-50, 0}, {-50, 2}, {-48, 2}}, color = {0, 0, 127}));
  connect(add2.u1, gain1.y) annotation(
    Line(points = {{-74, -58}, {-74, -84}, {-27, -84}}, color = {0, 0, 127}));
  connect(add2.u2, gain2.y) annotation(
    Line(points = {{-62, -58}, {-28, -58}}, color = {0, 0, 127}));
  connect(add2.y, add1.u2) annotation(
    Line(points = {{-68, -35}, {-68, -12}, {-48, -12}, {-48, -10}}, color = {0, 0, 127}));
  connect(linear_WindTrim1.alpha, transferFunction2.u) annotation(
    Line(points = {{92, 2}, {92, 2}, {92, -56}, {44, -56}, {44, -56}, {42, -56}}, color = {0, 0, 127}));
  connect(linear_WindTrim1.q, transferFunction1.u) annotation(
    Line(points = {{92, 10}, {100, 10}, {100, -84}, {46, -84}, {46, -84}}, color = {0, 0, 127}));
  connect(const.y, linear_WindTrim1.thrust) annotation(
    Line(points = {{48, 68}, {62, 68}, {62, 10}, {70, 10}, {70, 10}}, color = {0, 0, 127}));
  connect(add3.y, linear_WindTrim1.deltaE) annotation(
    Line(points = {{54, 2}, {68, 2}, {68, 2}, {70, 2}}, color = {0, 0, 127}));
  connect(gain3.y, add3.u1) annotation(
    Line(points = {{10, 44}, {18, 44}, {18, 8}, {30, 8}}, color = {0, 0, 127}));
  connect(PI.y, add3.u2) annotation(
    Line(points = {{15, -4}, {30, -4}}, color = {0, 0, 127}));
  connect(add1.y, gain3.u) annotation(
    Line(points = {{-24, -4}, {-16, -4}, {-16, 44}, {-8, 44}, {-8, 44}}, color = {0, 0, 127}));
  connect(add1.y, PI.u) annotation(
    Line(points = {{-25, -4}, {-8, -4}}, color = {0, 0, 127}));
  connect(gain2.u, transferFunction2.y) annotation(
    Line(points = {{-6, -58}, {19, -58}, {19, -56}}, color = {0, 0, 127}));
  connect(gain1.u, transferFunction1.y) annotation(
    Line(points = {{-4, -84}, {22, -84}, {22, -84}, {22, -84}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end FlightControlModel;