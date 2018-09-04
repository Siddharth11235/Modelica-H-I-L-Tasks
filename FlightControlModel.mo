model FlightControlModel
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-4, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-82, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {28, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 0.005)  annotation(
    Placement(visible = true, transformation(origin = {23, -17}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 20, k = 0.02)  annotation(
    Placement(visible = true, transformation(origin = {-46, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-124, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(const.y, feedback2.u1) annotation(
    Line(points = {{-113, 12}, {-90, 12}}, color = {0, 0, 127}));
  connect(flightLongs1.theta, feedback2.u2) annotation(
    Line(points = {{40, 16}, {70, 16}, {70, -40}, {-80, -40}, {-80, 4}, {-82, 4}}, color = {0, 0, 127}));
  connect(flightLongs1.q, gain2.u) annotation(
    Line(points = {{40, 8}, {48, 8}, {48, -16}, {30, -16}, {30, -16}}, color = {0, 0, 127}));
  connect(PI.u, feedback2.y) annotation(
    Line(points = {{-58, 12}, {-72, 12}}, color = {0, 0, 127}));
  connect(PI.y, feedback1.u1) annotation(
    Line(points = {{-35, 12}, {-12, 12}}, color = {0, 0, 127}));
  connect(gain2.y, feedback1.u2) annotation(
    Line(points = {{17.5, -17}, {-4, -17}, {-4, 4}}, color = {0, 0, 127}));
  connect(flightLongs1.del, feedback1.y) annotation(
    Line(points = {{18, 12}, {6, 12}, {6, 12}, {6, 12}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));end FlightControlModel;