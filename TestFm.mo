model TestFm
  Real del[3] = {0,-0.0298367,0};
  Real Thrust[3] = {1527.18 , 0, 0};
parameter Real m = 1043.26;
parameter Real s = 16.1651;//reference area
parameter Real cBar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.8};//gravitational force




  
  ForceMoment_Gen forceMoment_Gen1( W =W,  b= b, cbar = cBar, s = s)  annotation(
    Placement(visible = true, transformation(origin = {-62, -12}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  Flight6DOF flight6DOF1(J = {{1285.31, 0, 0}, {0, 1824.93, 0}, {0, 0, 2666.893}}, angles(start = {0.0, 0, 0.0}), g = {0,0, 9.8}, mass = m, omega( fixed = true,start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {60*cos(0.0101061), 0, 60*sin(0.0101061)}))  annotation(
    Placement(visible = true, transformation(origin = {36, -12}, extent = {{-26, -26}, {26, 26}}, rotation = 0))); 
    
initial equation
forceMoment_Gen1.alpha= 0.0101061;
forceMoment_Gen1.Cm = 0;
    
 
 equation
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{65, -20}, {92, -20}, {92, 34}, {-49, 34}, {-49, 17}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{-33, 1}, {8, 1}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Moment, flight6DOF1.Moment) annotation(
    Line(points = {{-33, -25}, {8, -25}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.vel, flight6DOF1.v) annotation(
    Line(points = {{-62, 17}, {-62, 50}, {65, 50}, {65, 11}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{65, -35}, {96, -35}, {96, 72}, {-74, 72}, {-74, 47}, {-74.5, 47}, {-74.5, 17}, {-75, 17}}, color = {0, 0, 127}, thickness = 0.5));
     
  forceMoment_Gen1.Thrust = Thrust;
  forceMoment_Gen1.delta = del;
  annotation(
    uses(Modelica(version = "3.2.2")));end TestFm;