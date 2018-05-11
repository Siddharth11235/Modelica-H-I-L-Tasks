model TestFm
  Real del[3] = {0,-0.0156543,0};
  Real Thrust[3] = {2257.12 , 0, 0};
  Real alpha = 2.08534e-5;
  ForceMoment_Gen forceMoment_Gen1(Force(start = {0, 0, 0}), W = 1043.26 * {0, 0, 9.8}, alpha(displayUnit = "rad",fixed = true, start = -0.027113))  annotation(
    Placement(visible = true, transformation(origin = {-62, -12}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  Flight6DOF flight6DOF1(J = {{1285.31, 0, 0}, {0, 1824.93, 0}, {0, 0, 2666.893}}, angles(start = {0, 0, 0}), g = {0, 0, 9.8}, mass = 1043.26, omega(start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {65, 0, 0}))  annotation(
    Placement(visible = true, transformation(origin = {26, -12}, extent = {{-26, -26}, {26, 26}}, rotation = 0))); 
    
     equation
  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{55, -35}, {96, -35}, {96, 72}, {-74, 72}, {-74, 47}, {-74.5, 47}, {-74.5, 17}, {-75, 17}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.vel, flight6DOF1.v) annotation(
    Line(points = {{-62, 17}, {-62, 50}, {55, 50}, {55, 11}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Moment, flight6DOF1.Moment) annotation(
    Line(points = {{-33, -25}, {-2, -25}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{-33, 1}, {-2, 1}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{55, -20}, {92, -20}, {92, 34}, {-49, 34}, {-49, 17}}, color = {0, 0, 127}, thickness = 0.5));
     
  forceMoment_Gen1.Thrust = Thrust;
  forceMoment_Gen1.delta = del;
  forceMoment_Gen1.alpha = alpha;
  annotation(
    uses(Modelica(version = "3.2.2")));end TestFm;