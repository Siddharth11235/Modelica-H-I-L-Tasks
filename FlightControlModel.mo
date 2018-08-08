model FlightControlModel

  
  Modelica.Blocks.Sources.Constant const(k = 1112.82)  annotation(
    Placement(visible = true, transformation(origin = {-65, 87}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = -0.15625)  annotation(
    Placement(visible = true, transformation(origin = {-90, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  linear_WindTrim linear_WindTrim1 annotation(
    Placement(visible = true, transformation(origin = {-4, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1 annotation(
    Placement(visible = true, transformation(origin = {38, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real[4] k_place = [77.19313887883463,	6075.030661023554,	122.31655064066875,	-6717.298968943441;
-0.2891312664172394,	-22.733145450996215,	-0.5318537635280807,	25.07093397939501];
  Modelica.Blocks.Math.Add add1 annotation(
    Placement(visible = true, transformation(origin = {-46, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
algorithm

  equation
  connect(const1.y, add1.u1) annotation(
    Line(points = {{-78, 58}, {-58, 58}, {-58, 58}, {-58, 58}}, color = {0, 0, 127}));
  connect(add1.y, linear_WindTrim1.deltaE) annotation(
    Line(points = {{-34, 52}, {-14, 52}, {-14, 52}, {-14, 52}}, color = {0, 0, 127}));
  connect(const.y, linear_WindTrim1.thrust) annotation(
    Line(points = {{-56, 88}, {-30, 88}, {-30, 59}, {-15, 59}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end FlightControlModel;