model FlightControlModel
  linear_FlightLongs linear_FlightLongs1 annotation(
    Placement(visible = true, transformation(origin = {78, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {48, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add31(k3 = -1)  annotation(
    Placement(visible = true, transformation(origin = {-14, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-82, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator1(k = 0.5)  annotation(
    Placement(visible = true, transformation(origin = {-48, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {-120, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = 0.05)  annotation(
    Placement(visible = true, transformation(origin = {18, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 0.2) annotation(
    Placement(visible = true, transformation(origin = {69, -13}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0.1) annotation(
    Placement(visible = true, transformation(origin = {-56, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(const1.y, add31.u1) annotation(
    Line(points = {{-44, 68}, {-32, 68}, {-32, 20}, {-26, 20}, {-26, 20}}, color = {0, 0, 127}));
  connect(const.y, feedback2.u1) annotation(
    Line(points = {{-108, 12}, {-92, 12}, {-92, 12}, {-90, 12}}, color = {0, 0, 127}));
  connect(integrator1.y, add31.u2) annotation(
    Line(points = {{-36, 12}, {-28, 12}, {-28, 12}, {-26, 12}}, color = {0, 0, 127}));
  connect(add31.y, gain1.u) annotation(
    Line(points = {{-3, 12}, {6, 12}}, color = {0, 0, 127}));
  connect(linear_FlightLongs1.q, add31.u3) annotation(
    Line(points = {{90, 16}, {102, 16}, {102, -38}, {-34, -38}, {-34, 4}, {-26, 4}}, color = {0, 0, 127}));
  connect(linear_FlightLongs1.alpha, gain2.u) annotation(
    Line(points = {{90, 8}, {94, 8}, {94, -14}, {80, -14}, {80, -12}}, color = {0, 0, 127}));
  connect(feedback1.u2, gain2.y) annotation(
    Line(points = {{48, 4}, {48, 4}, {48, -14}, {60, -14}, {60, -12}}, color = {0, 0, 127}));
  connect(gain1.y, feedback1.u1) annotation(
    Line(points = {{30, 12}, {38, 12}, {38, 12}, {40, 12}}, color = {0, 0, 127}));
  connect(integrator1.u, feedback2.y) annotation(
    Line(points = {{-60, 12}, {-74, 12}, {-74, 12}, {-72, 12}}, color = {0, 0, 127}));
  connect(feedback1.y, linear_FlightLongs1.u[1]) annotation(
    Line(points = {{58, 12}, {66, 12}, {66, 12}, {68, 12}}, color = {0, 0, 127}));
  connect(feedback2.u2, linear_FlightLongs1.q) annotation(
    Line(points = {{-82, 4}, {-82, 4}, {-82, -38}, {102, -38}, {102, 16}, {90, 16}, {90, 16}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));end FlightControlModel;