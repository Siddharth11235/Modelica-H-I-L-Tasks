model TestFm
parameter  Real del[3] = {0,-0.0304977268414434,0};
parameter  Real thrust[3] = {1526.49255280348 , 0, 0};
parameter Real m = 1043.26;
parameter Real s = 16.1651;//reference area
parameter Real cBar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.8};//gravitational force




  
  ForceMoment_Gen forceMoment_Gen1( W =W,  b= b, cbar = cBar, s = s)  annotation(
    Placement(visible = true, transformation(origin = {-56, -12}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  Flight6DOF flight6DOF1(J = {{1285.31, 0, 0}, {0, 1824.93, 0}, {0, 0, 2666.893}}, g = {0,0, 9.8}, mass = m, omega( fixed = true,start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {60*cos(0.0105761613094709), 0, 60*sin(0.0105761613094709)}))  annotation(
    Placement(visible = true, transformation(origin = {36, -12}, extent = {{-26, -26}, {26, 26}}, rotation = 0))); 
    
initial equation
forceMoment_Gen1.alpha = 0.0105761613094709;
forceMoment_Gen1.angles[2] = 0.0105761613094709;
 
 equation
  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{64, -36}, {102, -36}, {102, 54}, {-70, 54}, {-70, 16}, {-68, 16}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{64, -20}, {98, -20}, {98, 32}, {-42, 32}, {-42, 16}, {-42, 16}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.v, forceMoment_Gen1.vel) annotation(
    Line(points = {{64, 12}, {90, 12}, {90, 42}, {-58, 42}, {-58, 16}, {-56, 16}}, color = {0, 0, 127}, thickness = 0.5));
  
  connect(flight6DOF1.Moment, forceMoment_Gen1.Moment) annotation(
    Line(points = {{8, -24}, {-28, -24}, {-28, -20}, {-28, -20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{-28, -4}, {6, -4}, {6, 2}, {8, 2}}, color = {0, 0, 127}, thickness = 0.5));
     
  forceMoment_Gen1.thrust = thrust;
  forceMoment_Gen1.delta = del;
  annotation(
    uses(Modelica(version = "3.2.2")));end TestFm;