model TestFm



parameter Real m = 1043.26;//1.56 for zagi
parameter Real s = 16.1651;//reference area
parameter Real cbar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.81};//gravitational force
//parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;



parameter Real CD0    = 0.036;//= 0.01631;for Zagi
parameter Real K_drag  = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_beta  = -0.31;//for cessna
parameter Real Cy_p  = -0.037;//for cessna
parameter Real Cy_r   = 0.21;//for cessna
parameter Real Cy_delta_r = 0.187; //for cessna
parameter Real Cy_delta_a= 0;     //for cessna

// lift
parameter Real CL0 = 0.25;   //for cessna
parameter Real CL_alpha = 4.47;//for cessna
parameter Real CL_q = 3.9;//for cessna
parameter Real CL_delta_e = 0.3476;//for cessna

// rolling moment
parameter Real Cl_beta = -0.089;//for cessna
parameter Real Cl_p = -0.47;//for cessna
parameter Real Cl_r = 0.096;//for cessna
parameter Real Cl_delta_a= -0.09;//for cessna
parameter Real Cl_delta_r = 0.0147;//for cessna

// pitching moment
parameter Real Cm0 = -0.02;//for cessna
parameter Real Cm_alpha = -1.8;//for cessna
parameter Real Cm_q   = -12.4;//for cessna
parameter Real Cm_delta_e = -1.28;//for cessna

// yawing moment
parameter Real Cn_beta = 0.065;//for cessna
parameter Real Cn_p  = -0.03;//for cessna
parameter Real Cn_r = -0.99;//for cessna
parameter Real Cn_delta_a = -0.0053;//for cessna
parameter Real Cn_delta_r = -0.0657;//for cessna


//Initial conditions. (delta[2], thrust[1] and the others are straightforward)
parameter  Real del[3] = {0,-0.15625,0};
parameter  Real thrust[3] = {1112.82 , 0, 0};
parameter Real alphazero = 0.1;
parameter Real V = 39.8858;

  
  ForceMoment_Gen forceMoment_Gen1( CD0 = CD0, CD_alpha = CD_alpha, CD_beta = CD_beta, CD_delta_e = CD_delta_e, CD_q = CD_q, CL0 = CL0, CL_alpha = CL_alpha, CL_delta_e = CL_delta_e, CL_q = CL_q, Cl_beta = Cl_beta, Cl_delta_a = Cl_delta_a, Cl_delta_r =Cl_delta_r, Cl_p = Cl_p, Cl_r = Cl_r, Cm0 = Cm0, Cm_alpha = Cm_alpha, Cm_delta_e = Cm_delta_e, Cm_q = Cm_q, Cn_beta = Cn_beta, Cn_delta_a = Cn_delta_a, Cn_delta_r = Cn_delta_r, Cn_p = Cn_p, Cn_r = Cn_r, Cy_beta = Cy_beta, Cy_delta_a = Cy_delta_a, Cy_delta_r = Cy_delta_r, Cy_p = Cy_p, Cy_r = Cy_r,W =m * {0, 0, 9.81},  b= b, cbar =cbar, g = 9.81, m = m, rho = 1.225, s = s)  annotation(
    Placement(visible = true, transformation(origin = {-9, 1}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));



  Flight6DOF flight6DOF1(J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}}, g = {0,0, 9.81},mass = m, omega( fixed = true,start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {V*cos(alphazero), 0, V*sin(alphazero)}))  annotation(
    Placement(visible = true, transformation(origin = {51, 1}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
  
Modelica.Blocks.Sources.RealExpression[3] realExpression1(y = if time > 10 and time < 17 then {0, del[2] + 3.1412 / 36, 0} else del)  annotation(    Placement(visible = true, transformation(origin = {-85, -4}, extent = {{-25, -14}, {25, 14}}, rotation = 0)));



initial equation
forceMoment_Gen1.alpha = alphazero;
forceMoment_Gen1.angles[2] = alphazero;

equation

  connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{61, -7}, {102, -7}, {102, 54}, {-70, 54}, {-70, 20}, {-17.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{61, -2}, {98, -2}, {98, 32}, {-0.5, 32}, {-0.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.v, forceMoment_Gen1.vel) annotation(
    Line(points = {{61, 9}, {90, 9}, {90, 42}, {-58, 42}, {-58, 20}, {-9, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(flight6DOF1.Moment, forceMoment_Gen1.Moment) annotation(
    Line(points = {{41, -3}, {21.5, -3}, {21.5, -4}, {10, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{10, 6}, {41, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceMoment_Gen1.delta, realExpression1.y) annotation(    Line(points = {{-28, -4}, {-57.5, -4}}, color = {0, 0, 127}, thickness = 0.5));//Comment this line and line 70 for trim conditions.
//forceMoment_Gen1.delta = del;//Uncomment this line for trim conditions.
  forceMoment_Gen1.thrust = thrust;
  annotation(
    uses(Modelica(version = "3.2.2")));
    end TestFm;