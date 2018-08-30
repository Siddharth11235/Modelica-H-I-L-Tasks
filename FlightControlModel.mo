model FlightControlModel
  linear_FlightLongs linear_FlightLongs1 annotation(
    Placement(visible = true, transformation(origin = {78, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {26, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-82, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-120, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 5, k = 0.01)  annotation(
    Placement(visible = true, transformation(origin = {-32, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2 annotation(
    Placement(visible = true, transformation(origin = {49, -17}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {10, 0}, b = {1, 0}) annotation(
    Placement(visible = true, transformation(origin = {69, -17}, extent = {{7, -7}, {-7, 7}}, rotation = 0)));
equation
  connect(transferFunction1.y, gain2.u) annotation(
    Line(points = {{61, -17}, {55, -17}}, color = {0, 0, 127}));
  connect(linear_FlightLongs1.alpha, transferFunction1.u) annotation(
    Line(points = {{90, 8}, {92, 8}, {92, -18}, {78, -18}, {78, -16}}, color = {0, 0, 127}));
  connect(gain2.y, feedback1.u2) annotation(
    Line(points = {{43.5, -17}, {26, -17}, {26, 4}}, color = {0, 0, 127}));
  connect(PI.y, feedback1.u1) annotation(
    Line(points = {{-20, 12}, {18, 12}}, color = {0, 0, 127}));
  connect(feedback1.y, linear_FlightLongs1.u[1]) annotation(
    Line(points = {{35, 12}, {68, 12}}, color = {0, 0, 127}));
  connect(feedback2.y, PI.u) annotation(
    Line(points = {{-72, 12}, {-44, 12}, {-44, 12}, {-44, 12}}, color = {0, 0, 127}));
  connect(const.y, feedback2.u1) annotation(
    Line(points = {{-108, 12}, {-92, 12}, {-92, 12}, {-90, 12}}, color = {0, 0, 127}));
  connect(feedback2.u2, linear_FlightLongs1.q) annotation(
    Line(points = {{-82, 4}, {-82, 4}, {-82, -38}, {102, -38}, {102, 16}, {90, 16}, {90, 16}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));end FlightControlModel;