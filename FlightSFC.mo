model FlightSFC
  import Modelica.Blocks.Interfaces.*;

  block Mat_mulA
    RealInput x[4, 1] annotation(
      Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Real[4, 4] A = 1.0e+03 * [-0.2956, -1.5455, 0, -0.0098; -0.0003, -0.0018, 0.0010, 0; -0.0860, -0.0250, 0, 0; 0, 0, 0.0010, 0];
    RealOutput[4, 1] Ax annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    Ax = A * x;
  end Mat_mulA;

  block Mat_mulB
    RealInput u[2, 1] annotation(
      Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Real B[4, 2] = [-120.1805, 0.0010; -0.1366, -0.0000; -17.7834, 0; 0, 0];
    RealOutput[4, 1] Bu annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    Bu = B * u;
  end Mat_mulB;

  block Mat_mulC
    RealInput x[4, 1] annotation(
      Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Real C[4, 4] = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    RealOutput[4, 1] y annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    y = C * x;
  end Mat_mulC;

  parameter Real C[4, 4] = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];

  block Mat_mulK
    RealInput x[4, 1] annotation(
      Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Real K[2, 4] = 1.0e+06 * [0.0000, 0.0000, -0.0000, -0.0000; 0.3019, -1.4548, 0.0015, -0.0110];
    RealOutput[2, 1] Kx annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    Kx = K * x;
  end Mat_mulK;

  Modelica.Blocks.Continuous.TransferFunction[4,1] transferFunction1 annotation(
    Placement(visible = true, transformation(origin = {40, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1[2,1] annotation(
    Placement(visible = true, transformation(origin = {-66, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1[4,1] annotation(
    Placement(visible = true, transformation(origin = { -2, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightSFC.Mat_mulB mat_mulB1 annotation(
    Placement(visible = true, transformation(origin = {-38, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightSFC.Mat_mulA mat_mulA1 annotation(
    Placement(visible = true, transformation(origin = {6, -32}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  FlightSFC.Mat_mulK mat_mulK1 annotation(
    Placement(visible = true, transformation(origin = {20, -68}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  FlightSFC.Mat_mulC mat_mulC1 annotation(
    Placement(visible = true, transformation(origin = {82, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const[2,1](k = [-0.15625; 1112.82])  annotation(
    Placement(visible = true, transformation(origin = {-106, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(const.y, feedback1.u1) annotation(
    Line(points = {{-95, 8}, {-74, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(transferFunction1.y, mat_mulC1.x) annotation(
    Line(points = {{52, 2}, {68, 2}, {68, 2}, {72, 2}}, color = {0, 0, 127}, thickness = 0.5));
  connect(transferFunction1.y, mat_mulK1.x) annotation(
    Line(points = {{51, 2}, {61, 2}, {61, -68}, {33, -68}, {33, -68}, {31, -68}, {31, -68}}, color = {0, 0, 127}, thickness = 0.5));
  connect(feedback1.u2, mat_mulK1.Kx) annotation(
    Line(points = {{-66, 0}, {-66, 0}, {-66, 0}, {-66, 0}, {-66, -68}, {10, -68}, {10, -68}, {10, -68}, {10, -68}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add1.u2, mat_mulA1.Ax) annotation(
    Line(points = {{-14, -4}, {-15, -4}, {-15, -4}, {-16, -4}, {-16, -32}, {-4, -32}, {-4, -32}, {-4, -32}, {-4, -32}}, color = {0, 0, 127}, thickness = 0.5));
  connect(transferFunction1.y, mat_mulA1.x) annotation(
    Line(points = {{51, 2}, {61, 2}, {61, -34}, {19, -34}, {19, -33}, {17, -33}, {17, -32}}, color = {0, 0, 127}, thickness = 0.5));
  connect(mat_mulB1.Bu, add1.u1) annotation(
    Line(points = {{-27, 8}, {-13, 8}, {-13, 8}, {-15, 8}, {-15, 8}, {-15, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(feedback1.y, mat_mulB1.u) annotation(
    Line(points = {{-57, 8}, {-53, 8}, {-53, 8}, {-49, 8}, {-49, 8}, {-50, 8}, {-50, 8}, {-49, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add1.y, transferFunction1.u) annotation(
    Line(points = {{9, 2}, {17, 2}, {17, 2}, {27, 2}, {27, 2}, {27, 2}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    uses(Modelica(version = "3.2.2")));
end FlightSFC;