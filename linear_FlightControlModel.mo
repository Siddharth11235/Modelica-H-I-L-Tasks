model linear_FlightControlModel
  parameter Integer n = 13; // states
  parameter Integer k = 0; // top-level inputs
  parameter Integer l = 0; // top-level outputs
  parameter Real x0[13] = {0,39.8858,0,0,0,0,0,0,0,0,0,0,100};
  parameter Real u0[0] = {i for i in 1:0};
  parameter Real A[13,13] = [-0,-0,-1,-0,-0,-0,-0,-0,-1,-0,-0,-0,-0;-0.1705448169971296,-0.0296619355813421,-2.022594803190353,-0,-0,-9.81,-0,-0,0.134732298020368,-0,-0,-0,-0;-0.1315800379614956,-0.008023594296665741,-1.58723101124695,0,0,0,0,0,1.103949690177087,0,0,0,0;0,0,0,-0.1292592588636269,0,0,0.2459521935124781,-0.001915703163996151,0,-0.9891270901502921,0,0,0;0,0,0,0,0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,1,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0,0,0;0,0,0,-11.90057043291414,0,0,0,-8.595910464397949,0,1.755760435281283,0,0,0;-16.4947094455534,0.1163104922444052,-6.700975712256071,0,0,0,0,0,13.50404360524907,0,0,0,0;0,0,0,4.188836643431083,0,0,0,-0.2644341708065558,0,-8.726327636616343,0,0,0;0,1,0,0,0,0,0,0,0,0,0,0,0;0,0,0,0,39.8858,0,0,0,0,0,0,0,0;-0,-0,-0,-0,-0,-39.8858,-0,-0,-0,-0,-0,-0,-0];
  parameter Real B[13,0] = zeros(13,0);
  parameter Real C[0,13] = zeros(0,13);
  parameter Real D[0,0] = zeros(0,0);
  Real x[13](start=x0);
  input Real u[0];
  output Real y[0];

  Real 'x_PI.x' = x[1];
Real 'x_wind6DOFBlock1.V' = x[2];
Real 'x_wind6DOFBlock1.alpha' = x[3];
Real 'x_wind6DOFBlock1.beta' = x[4];
Real 'x_wind6DOFBlock1.chi' = x[5];
Real 'x_wind6DOFBlock1.gamma' = x[6];
Real 'x_wind6DOFBlock1.mu' = x[7];
Real 'x_wind6DOFBlock1.p' = x[8];
Real 'x_wind6DOFBlock1.q' = x[9];
Real 'x_wind6DOFBlock1.r' = x[10];
Real 'x_wind6DOFBlock1.x' = x[11];
Real 'x_wind6DOFBlock1.y' = x[12];
Real 'x_wind6DOFBlock1.z' = x[13];

equation
  der(x) = A * x + B * u;
  y = C * x + D * u;
end linear_FlightControlModel;
