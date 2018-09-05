model FlightControlModel
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {26, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-52, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {58, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 0.005)  annotation(
    Placement(visible = true, transformation(origin = {53, -17}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 15, k = 0.2)  annotation(
    Placement(visible = true, transformation(origin = {-14, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-94, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(PI.y, feedback1.u1) annotation(
    Line(points = {{-3, 12}, {18, 12}}, color = {0, 0, 127}));
  connect(PI.u, feedback2.y) annotation(
    Line(points = {{-26, 12}, {-42, 12}}, color = {0, 0, 127}));
  connect(const.y, feedback2.u1) annotation(
    Line(points = {{-83, 12}, {-60, 12}}, color = {0, 0, 127}));
  connect(gain2.y, feedback1.u2) annotation(
    Line(points = {{47.5, -17}, {26, -17}, {26, -6.5}, {26, -6.5}, {26, 4}}, color = {0, 0, 127}));
  connect(flightLongs1.q, gain2.u) annotation(
    Line(points = {{69, 8.7}, {79, 8.7}, {79, -15.3}, {61, -15.3}, {61, -15.3}, {59, -15.3}, {59, -15.3}}, color = {0, 0, 127}));
  connect(flightLongs1.del, feedback1.y) annotation(
    Line(points = {{47, 12}, {37, 12}, {37, 12}, {35, 12}}, color = {0, 0, 127}));
  connect(flightLongs1.theta, feedback2.u2) annotation(
    Line(points = {{69, 15.3}, {85, 15.3}, {85, 15.3}, {101, 15.3}, {101, -40.7}, {-49, -40.7}, {-49, 3.3}, {-51, 3.3}, {-51, 3.3}, {-53, 3.3}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));end FlightControlModel;