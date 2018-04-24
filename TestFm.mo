model TestFm
  Real al = 1;
  Real be = 1;
  Real del[3] = {0.0,0,0};
  Real Thrust[3] = {0 , 1, -9.8};
  ForceMoment_Gen forceMoment_Gen1 annotation(
    Placement(visible = true, transformation(origin = {-14, -20}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  Flight6DOF flight6DOF1(J = {{1285.31, 0, 0}, {0, 1824.93, 0}, {0, 0, 2666.893}}, mass = 1043.26)  annotation(
    Placement(visible = true, transformation(origin = {60, -7.10543e-15}, extent = {{-26, -26}, {26, 26}}, rotation = 0))); 
    
     equation
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{88, -8}, {92, -8}, {92, 34}, {0, 34}, {0, 8}, {0, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{15, -7}, {22.5, -7}, {22.5, 13}, {32, 13}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Moment, flight6DOF1.Moment) annotation(
    Line(points = {{15, -33}, {22.5, -33}, {22.5, -13}, {32, -13}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{88, -24}, {96, -24}, {96, 58}, {-26, 58}, {-26, 9}, {-27, 9}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.vel, flight6DOF1.v) annotation(
    Line(points = {{-14, 9}, {-14, 40}, {88, 40}, {88, 24}}, color = {0, 0, 127}, thickness = 0.5));
     
  forceMoment_Gen1.alpha = al;
  forceMoment_Gen1.beta = be;
  forceMoment_Gen1.Thrust = Thrust;
  forceMoment_Gen1.delta = del;
  annotation(
    uses(Modelica(version = "3.2.2")));end TestFm;