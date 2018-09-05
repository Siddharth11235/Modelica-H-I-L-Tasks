model FlightPitchHold
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {26, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-52, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {68, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 0.005)  annotation(
    Placement(visible = true, transformation(origin = {53, -17}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 15, k = 0.2)  annotation(
    Placement(visible = true, transformation(origin = {-14, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant pitch_angle(k = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-94, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 1112.82) annotation(
    Placement(visible = true, transformation(origin = {24, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(const.y, flightLongs1.thrust) annotation(
    Line(points = {{36, 58}, {48, 58}, {48, 20}, {58, 20}, {58, 20}}, color = {0, 0, 127}));
  connect(flightLongs1.q, gain2.u) annotation(
    Line(points = {{80, 12}, {86, 12}, {86, -16}, {60, -16}, {60, -16}}, color = {0, 0, 127}));
  connect(feedback1.y, flightLongs1.del) annotation(
    Line(points = {{36, 12}, {56, 12}, {56, 12}, {58, 12}}, color = {0, 0, 127}));
  connect(flightLongs1.theta, feedback2.u2) annotation(
    Line(points = {{79, 16}, {101, 16}, {101, -40.7}, {-49, -40.7}, {-49, 3.3}, {-53, 3.3}}, color = {0, 0, 127}));
  connect(PI.y, feedback1.u1) annotation(
    Line(points = {{-3, 12}, {18, 12}}, color = {0, 0, 127}));
  connect(PI.u, feedback2.y) annotation(
    Line(points = {{-26, 12}, {-42, 12}}, color = {0, 0, 127}));
  connect(pitch_angle.y, feedback2.u1) annotation(
    Line(points = {{-83, 12}, {-60, 12}}, color = {0, 0, 127}));
  connect(gain2.y, feedback1.u2) annotation(
    Line(points = {{47.5, -17}, {26, -17}, {26, -6.5}, {26, -6.5}, {26, 4}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end FlightPitchHold;