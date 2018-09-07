model Wind6DOFSplit
  Wind6DOFBasic wind6DOFBasic1 annotation(
    Placement(visible = true, transformation(origin = {28, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  WindForceMoment windForceMoment1 annotation(
    Placement(visible = true, transformation(origin = {-12, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant thrust(k = 1112.82)  annotation(
    Placement(visible = true, transformation(origin = {-76, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression del[3](y = if time > 100 and time < 101 then {(-0.15625) + 3.1412 / 180, 0, 0} else {-0.15625, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-86, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(del.y, windForceMoment1.delta) annotation(
    Line(points = {{-75, 0}, {-22, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(thrust.y, windForceMoment1.thrust) annotation(
    Line(points = {{-64, 40}, {-46, 40}, {-46, 4}, {-22, 4}, {-22, 4}}, color = {0, 0, 127}));
  connect(windForceMoment1.Moment, wind6DOFBasic1.Moment) annotation(
    Line(points = {{0, 0}, {16, 0}, {16, 0}, {18, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(windForceMoment1.Force, wind6DOFBasic1.Force) annotation(
    Line(points = {{0, 8}, {16, 8}, {16, 8}, {18, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(windForceMoment1.omega, wind6DOFBasic1.omega) annotation(
    Line(points = {{-8, 16}, {-8, 16}, {-8, 48}, {60, 48}, {60, 0}, {40, 0}, {40, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(windForceMoment1.VAB, wind6DOFBasic1.VAB) annotation(
    Line(points = {{-16, 16}, {-16, 16}, {-16, 40}, {56, 40}, {56, 8}, {40, 8}, {40, 8}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    uses(Modelica(version = "3.2.2")));end Wind6DOFSplit;