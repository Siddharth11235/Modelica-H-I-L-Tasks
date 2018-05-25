package aerospace
  extends Modelica.Icons.Package;

  class Components
    block SixDOFEoM
      extends Modelica.Blocks.Interfaces.BlockIcon;
      import SI = Modelica.SIunits;
      /* Properties of Aircraft */
      parameter SI.Mass m = 1043.26 "Mass of the aircraft";
      parameter SI.Inertia[3, 3] I = {{1285.31, 0, 0}, {0, 1824.93, 0}, {0, 0, 2666.893}} "Inertia";
      parameter SI.Length xe_init[3] = {0, 0, 0} "Initial position in Inertial axes {xe,ye,ze}";
      parameter SI.Velocity vb_init[3] = {0, 0, 0} "Initial velocity in body axes {u,v,w}";
      parameter Real Euler_init[3](final quantity = "Angle", final unit = "rad") = {0, 0, 0} "Initial Euler orientation {roll,pitch,yaw}";
      parameter SI.AngularVelocity omega_init[3] = {0, 0, 0} "Initial body rotation rates {p,q,r}";
      /* Inputs and outputs */
      Modelica.Blocks.Interfaces.RealInput Forces[3](each final quantity = "Force", final unit = "N") annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Moments[3](each final quantity = "AngularMomentum", final unit = "kg.m2/s") annotation(
        Placement(visible = true, transformation(origin = {-120, -38}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Vb[3](each final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {110, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Xe[3](each final quantity = "Length", final unit = "m") annotation(
        Placement(visible = true, transformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput EulerAngles[3](each final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput omega_b[3](each final quantity = "AngularVelocity", final unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {110, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput DCMbe[3, 3](unit = "1") annotation(
        Placement(visible = true, transformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VelocityAngularRatesEOM velocityAngularRatesEOM1(ub(start = vb_init[1]), vb(start = vb_init[2]), wb(start = vb_init[3]), p(start = omega_init[1]), q(start = omega_init[2]), r(start = omega_init[3])) annotation(
        Placement(visible = true, transformation(origin = {-16, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      EulerAnglesEOM eulerAnglesEOM1(phi(start = Euler_init[1]), theta(start = Euler_init[2]), psi(start = Euler_init[3])) annotation(
        Placement(visible = true, transformation(origin = {22, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      PositionRatesEoM positionRatesEoM1(xe(start = xe_init[1]), ye(start = xe_init[2]), ze(start = xe_init[3])) annotation(
        Placement(visible = true, transformation(origin = {-14, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      aerospace.Components.DCMCbeCalc dCMCbeCalc1 annotation(
        Placement(visible = true, transformation(origin = {8, -28}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    equation
/* Velocity and Angular rates */
      velocityAngularRatesEOM1.m = m;
      velocityAngularRatesEOM1.I = I;
/* Inputs */
      connect(Forces, velocityAngularRatesEOM1.Forces);
      connect(Moments, velocityAngularRatesEOM1.Moments);
/* Interconnections */
//connect(velocityAngularRatesEOM1.EulerAngles,eulerAnglesEOM1.EulerAngles);
      connect(eulerAnglesEOM1.omega_b, velocityAngularRatesEOM1.omega_b);
      connect(positionRatesEoM1.EulerAngles, eulerAnglesEOM1.EulerAngles);
      connect(positionRatesEoM1.Vb, velocityAngularRatesEOM1.Vb);
      connect(eulerAnglesEOM1.EulerAngles, dCMCbeCalc1.EulerAngles);
/* Outputs */
      connect(Vb, velocityAngularRatesEOM1.Vb);
      connect(omega_b, velocityAngularRatesEOM1.omega_b);
      connect(EulerAngles, eulerAnglesEOM1.EulerAngles);
      connect(Xe, positionRatesEoM1.Xe);
      connect(DCMbe, dCMCbeCalc1.DCMbe);
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5})));
    end SixDOFEoM;

    block VelocityAngularRatesEOM
      extends Modelica.Icons.InternalPackage;
      import SI = Modelica.SIunits;
      /* Properties of Aircraft */
      SI.Mass m "Mass of the aircraft";
      SI.Inertia[3, 3] I "Inertia";
      SI.Acceleration g = 9.81 "Acceleration due to gravity";
      Modelica.Blocks.Interfaces.RealInput Forces[3](each final quantity = "Force", final unit = "N") annotation(
        Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-108, 50}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Moments[3](each final quantity = "AngularMomentum", final unit = "kg.m2/s") annotation(
        Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-107, -51}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Vb[3](each final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput omega_b[3](each final quantity = "AngularVelocity", final unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      /*Modelica.Blocks.Interfaces.RealInput EulerAngles[3](each final quantity = "Angle", final unit = "rad") annotation(
                                                                                                                                                                                                                    Placement(visible = true, transformation(origin = {-120, 1.9984e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-108, 2.22045e-15}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
                                                                                                                                                                                                                */
    protected
      /* Declaratoin of individual variables for integration */
      SI.Velocity ub(start = 0) "Velocity in x-axis in body frame";
      SI.Velocity vb(start = 0) "Velocity in y-axis in body frame";
      SI.Velocity wb(start = 0) "Velocity in z-axis in body frame";
      SI.AngularVelocity p(start = 0) "Angular velocity about x-axis";
      SI.AngularVelocity q(start = 0) "Angular velocity about y-axis";
      SI.AngularVelocity r(start = 0) "Angular velocity about z-axis";
    equation
/* Equations of motion */
      der(ub) = Forces[1] / m + r * vb - q * wb;
// - (g * sin(EulerAngles[2]));
      der(vb) = Forces[2] / m - r * ub + p * wb;
// + (g * sin(EulerAngles[1]) * cos(EulerAngles[2]));
      der(wb) = Forces[3] / m + q * ub - p * vb;
// + (g * cos(EulerAngles[1]) * cos(EulerAngles[2]));
      der(p) = (I[3, 3] * Moments[1] + I[1, 3] * Moments[3] - (I[1, 3] * (I[2, 2] - I[1, 1] - I[3, 3]) * p + (I[1, 3] ^ 2 + I[3, 3] * (I[3, 3] - I[2, 2])) * r) * q) / (I[3, 3] * I[3, 3] - I[1, 3] ^ 2);
      der(q) = (Moments[2] - (I[1, 1] - I[3, 3]) * p * r - I[1, 3] * (p ^ 2 - r ^ 2)) / I[2, 2];
      der(r) = (I[1, 3] * Moments[1] + I[1, 1] * Moments[3] + (I[1, 3] * (I[2, 2] - I[1, 1] - I[3, 3]) * r + (I[1, 3] ^ 2 + I[1, 1] * (I[1, 1] - I[2, 2])) * p) * q) / (I[3, 3] * I[3, 3] - I[1, 3] ^ 2);
/* Grouping up the variables */
      Vb = {ub, vb, wb};
      omega_b = {p, q, r};
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5}), graphics = {Text(origin = {-3, 1}, extent = {{-21, 23}, {21, -23}}, textString = "u, v, w and p, q, r", fontSize = 45, fontName = "MS Shell Dlg 2")}));
    end VelocityAngularRatesEOM;

    block EulerAnglesEOM
      extends Modelica.Icons.InternalPackage;
      import SI = Modelica.SIunits;
      Modelica.Blocks.Interfaces.RealInput omega_b[3](each final quantity = "AngularVelocity", final unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {-106, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 1.33227e-15}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput EulerAngles[3](each final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      /* Declaratoin of individual variables for integration */
      Real phi(final quantity = "Angle", final unit = "rad", start = 0) "Euler Angle phi";
      Real theta(final quantity = "Angle", final unit = "rad", start = 0) "Euler Angle theta";
      Real psi(final quantity = "Angle", final unit = "rad", start = 0) "Euler Angle psi";
    equation
      der(phi) = omega_b[1] + (omega_b[2] * sin(phi) * tan(theta) + omega_b[3] * cos(phi)) * tan(theta);
      der(theta) = omega_b[2] * cos(phi) - omega_b[3] * sin(phi);
      der(psi) = (omega_b[2] * sin(phi) + omega_b[3] * cos(phi)) * tan(theta) / cos(theta);
/* Grouping up the variables */
      EulerAngles = {phi, theta, psi};
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5})));
    end EulerAnglesEOM;

    block PositionRatesEoM
      extends Modelica.Icons.InternalPackage;
      import SI = Modelica.SIunits;
      Modelica.Blocks.Interfaces.RealInput EulerAngles[3](each final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-108, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vb[3](each final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {-110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Xe[3](each final quantity = "AngularVelocity", final unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {106, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Velocity ub = Vb[1];
      SI.Velocity vb = Vb[2];
      SI.Velocity wb = Vb[3];
      Real phi(final quantity = "Angle", final unit = "rad") = EulerAngles[1];
      Real theta(final quantity = "Angle", final unit = "rad") = EulerAngles[2];
      Real psi(final quantity = "Angle", final unit = "rad") = EulerAngles[3];
      SI.Length xe(start = 0) "X-Position of Aircraft in inertial frame";
      SI.Length ye(start = 0) "Y-Position of Aircraft in inertial frame";
      SI.Length ze(start = 0) "Z-Position of Aircraft in inertial frame";
    equation
      der(xe) = cos(theta) * cos(psi) * ub + ((-cos(phi) * sin(psi)) + sin(phi) * sin(theta) * cos(psi)) * vb + (sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi)) * wb;
      der(ye) = cos(theta) * sin(psi) * ub + (cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)) * vb + ((-sin(phi) * cos(psi)) + cos(phi) * sin(theta) * sin(psi)) * wb;
      der(ze) = (-sin(theta) * ub) + sin(phi) * cos(theta) * vb + cos(phi) * cos(theta) * wb;
      Xe = {xe, ye, ze};
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5})));
    end PositionRatesEoM;

    block DCMCbeCalc
      extends Modelica.Blocks.Icons.Block;
      import SI = Modelica.SIunits;
      Modelica.Blocks.Interfaces.RealInput EulerAngles[3](final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 4.44089e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput DCMbe[3, 3](unit = "1") annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Real phi(final quantity = "Angle", final unit = "rad") = EulerAngles[1] "Euler Angle phi";
      Real theta(final quantity = "Angle", final unit = "rad") = EulerAngles[2] "Euler Angle theta";
      Real psi(final quantity = "Angle", final unit = "rad") = EulerAngles[3] "Euler Angle psi";
      SI.DimensionlessRatio C11(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C12(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C13(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C21(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C22(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C23(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C31(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C32(start = 0) "DCM Matrix";
      SI.DimensionlessRatio C33(start = 0) "DCM Matrix";
    equation
      C11 = cos(theta) * cos(psi);
      C12 = cos(theta) * sin(psi);
      C13 = -sin(theta);
      C21 = sin(theta) * sin(phi) * cos(psi) - sin(psi) * cos(phi);
      C22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
      C23 = sin(phi) * cos(theta);
      C31 = sin(theta) * cos(phi) * cos(psi) + sin(psi) * sin(phi);
      C32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
      C33 = cos(phi) * cos(theta);
      DCMbe = {{C11, C12, C13}, {C21, C22, C23}, {C31, C32, C33}};
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5}, initialScale = 0.1), graphics = {Text(origin = {-75, 0}, extent = {{-13, -6}, {13, 6}}, textString = "{phi,theta,psi}", fontName = "MS Shell Dlg 2"), Text(extent = {{60, 2}, {60, 2}}, textString = "text", fontName = "MS Shell Dlg 2"), Text(origin = {86, -5}, extent = {{-28, 9}, {12, -1}}, textString = "DCMbe", fontName = "MS Shell Dlg 2")}));
    end DCMCbeCalc;

    block IncidenceSideslipandAirspeed
      extends Modelica.Blocks.Icons.Block;
      import SI = Modelica.SIunits;
      Modelica.Blocks.Interfaces.RealInput Vb[3](each final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Airspeed(final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {110, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput alpha(final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput beta(final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {106, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      Airspeed = sqrt(abs(Vb[1] * Vb[1] + Vb[2] * Vb[2] + Vb[3] * Vb[1]));
      beta = asin(Vb[2] / Airspeed);
      alpha = atan2(Vb[3], Vb[1]);
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5})));
    end IncidenceSideslipandAirspeed;

    block DynamicPressure
      extends Modelica.Blocks.Icons.Block;
      import SI = Modelica.SIunits;
      Modelica.Blocks.Interfaces.RealInput Vb[3](each final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {-114, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput rho(final quantity = "Density", final unit = "kg/m3", min = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-112, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput qbar(final quantity = "Pressure", final unit = "Pa") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Velocity airspeed(start = 0);
    equation
      airspeed = sqrt(abs(Vb[1] * Vb[1] + Vb[2] * Vb[2] + Vb[3] * Vb[3]));
      qbar = 0.5 * rho * airspeed * airspeed;
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5})));
    end DynamicPressure;

    model AerodynamicForcesandMoments
      extends Modelica.Blocks.Icons.Block;
      import SI = Modelica.SIunits;
      parameter SI.Area S = 16.1651 "Reference area of the wing";
      parameter SI.Length b = 10.911 "Wingspan";
      parameter SI.Length cbar = 1.493 "Mean chord length of the wing";
      Modelica.Blocks.Interfaces.RealOutput Forces[3](each final quantity = "Force", final unit = "N") annotation(
        Placement(visible = true, transformation(origin = {110, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Moments[3](each final quantity = "AngularMomentum", final unit = "kg.m2/s") annotation(
        Placement(visible = true, transformation(origin = {110, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Coefficients[6](unit = "1") annotation(
        Placement(visible = true, transformation(origin = {-102, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-104, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput qbar(final quantity = "Pressure", final unit = "Pa", displayUnit = "bar") annotation(
        Placement(visible = true, transformation(origin = {-112, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-104, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Force L(start = 0) "Lift Force acting on the wing";
      SI.Force D(start = 0) "Drag force acting on the aircraft";
      SI.Force Y(start = 0) "Sideforce acting on the aircraft";
      SI.AngularMomentum l(start = 0) "Angular moment about x-axis in body frame";
      SI.AngularMomentum m(start = 0) "Angular moment about y-axis in body frame";
      SI.AngularMomentum n(start = 0) "Angular moment about z-axis in body frame";
    equation
      L = Coefficients[1] * qbar * S;
      D = Coefficients[2] * qbar * S;
      Y = Coefficients[3] * qbar * S;
      l = Coefficients[4] * qbar * S * b;
      m = Coefficients[5] * qbar * S * cbar;
      n = Coefficients[6] * qbar * S * b;
      Forces = {L, D, Y};
      Moments = {l, m, n};
      annotation(
        Icon(coordinateSystem(grid = {0.5, 0.5})));
    end AerodynamicForcesandMoments;

    class AerodynamicCoefficients
      extends Modelica.Blocks.Icons.Block;
      import SI = Modelica.SIunits;
      parameter SI.Length b = 10.911 "Wingspan of the aircraft";
      parameter SI.Length cbar = 1.493 "Standard mean chord";
      parameter SI.DimensionlessRatio CL0 = 0.25 "CL0";
      parameter Real CLalpha(unit = "/rad") = 4.7 "CLalpha";
      parameter Real CLq(unit = "/rad/s") = 1.7 "CLq";
      parameter Real CLdeltae(unit = "/rad") = 0.3476 "CLdelta_e";
      parameter SI.DimensionlessRatio CD0 = 0.036 "CD0";
      parameter Real CDbeta(unit = "/rad") = 0.17 "CDbeta";
      parameter Real CDdeltae(unit = "/rad") = 0.06 "CDdelta_e";
      parameter Real CYbeta(unit = "/rad") = -0.31 "CYbeta";
      parameter Real CYp(unit = "/rad/s") = -0.037 "CYp";
      parameter Real CYr(unit = "/rad/s") = 0.21 "CYr";
      parameter Real CYdeltaa(unit = "/rad") = 0.0 "CYdelta_a";
      parameter Real Clbeta(unit = "/rad") = -0.089 "Clbeta";
      parameter Real Clp(unit = "/rad/s") = -0.47 "Clp";
      parameter Real Clr(unit = "/rad/s") = 0.096 "Clr";
      parameter Real Cldeltaa(unit = "/rad") = -0.09 "Cldalta_a";
      parameter Real Cldeltar(unit = "/rad") = 0.0147 "Cldelta_r";
      parameter SI.DimensionlessRatio Cm0 = -0.02 "Cm0";
      parameter Real Cmalpha(unit = "/rad") = -1.8 "Cm_alpha";
      parameter Real Cmalphadot(unit = "/rad/sec") = -12.4 "Cm_alphadot";
      parameter Real Cmdeltae(unit = "/rad") = -1.28 "Cmdelta_e";
      parameter Real Cnbeta(unit = "/rad") = 0.065 "Cn_beta";
      parameter Real Cnp(unit = "/rad/s") = -0.03 "Cnp";
      parameter Real Cnr(unit = "/rad/s") = -0.99 "Cnr";
      parameter Real Cndeltaa(unit = "/rad") = -0.0053 "Cndelta_a";
      parameter Real Cndeltar(unit = "/rad") = -0.0657 "Cndelta_r";
      Modelica.Blocks.Interfaces.RealInput alpha(quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-107, 79}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput beta(quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-111, 65}, extent = {{-11, -11}, {11, 11}}, rotation = 0), iconTransformation(origin = {-107, 59}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Elevator(quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-112, 48}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-107, 21}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Aileron(quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-112, 32}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-107, 1}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Rudder(quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-110, 10}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-107, -21}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Airspeed(final quantity = "Velocity", final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {-115, -9}, extent = {{-15, -15}, {15, 15}}, rotation = 0), iconTransformation(origin = {-107, 41}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput omega_b[3](final quantity = "AngularVelocity", final unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {-113, -33}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-107, -81}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput CL(unit = "1") "Coefficient of Lift" annotation(
        Placement(visible = true, transformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput CD(unit = "1") "Coefficient of Drag" annotation(
        Placement(visible = true, transformation(origin = {110, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput CY(unit = "1") "Coefficient of Sideforce" annotation(
        Placement(visible = true, transformation(origin = {110, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Cl(unit = "1") "Coefficient of rolling moment" annotation(
        Placement(visible = true, transformation(origin = {112, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Cm(unit = "1") "Coefficient of pitching moment" annotation(
        Placement(visible = true, transformation(origin = {108, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Cn(unit = "1") "Coefficient of yawing moment" annotation(
        Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.AngularVelocity alphadot(start = 0);
    equation
//alphadot = der(alpha);
// UNCOMMENT THIS LINE IF ALPHADOT IS TO BE CALCULATED AND USED. CURRENTLY 0 IS BEING USED AS DEFAULT VALUE
      alphadot = 0.0;
//COMMENT THIS LINE IF THE ABOVE LINE IS UNCOMMENTED
      CL = CL0 + CLalpha * alpha + CLq * (omega_b[2] * cbar) / (2 * Airspeed) + CLdeltae * Elevator;
      CD = CD0 + 0.0830304 * CL * CL;
// + CDbeta * beta + CDdeltae * Elevator;
      CY = CYbeta * beta + CYp * (omega_b[1] * b) / (2 * Airspeed) + CYr * (omega_b[3] * b) / (2 * Airspeed) + CYdeltaa * Aileron;
      Cl = Clbeta * beta + Clp * (omega_b[1] * b) / (2 * Airspeed) + Clr * (omega_b[3] * b) / (2 * Airspeed) + Cldeltaa * Aileron + Cldeltaa * Rudder;
      Cm = Cm0 + Cmalpha * alpha + Cmalphadot * (alphadot * cbar) / (2 * Airspeed) + Cmdeltae * Elevator;
      Cn = Cnbeta * beta + Cnp * (omega_b[1] * b) / (2 * Airspeed) + Cnr * (omega_b[3] * b) / (2 * Airspeed) + Cndeltaa * Aileron + Cndeltaa * Rudder;
    end AerodynamicCoefficients;

    class ForceInBodyAxis
      extends Modelica.Blocks.Icons.Block;
      import SI = Modelica.SIunits;
      Modelica.Blocks.Interfaces.RealInput Forces_w[3](final quantity = "Force", final unit = "N") annotation(
        Placement(visible = true, transformation(origin = {-120, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput alpha(final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-120, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput beta(final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-120, -12}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput EulerAngles[3](each final quantity = "Angle", final unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Forces_b[3](final quantity = "Force", final unit = "N") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Thrust(final quantity = "Force", final unit = "N") annotation(
        Placement(visible = true, transformation(origin = {-120, -84}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Force Fx(start = 0);
      SI.Force Fy(start = 0);
      SI.Force Fz(start = 0);
      SI.Mass m = 1043.26;
      SI.Acceleration g = 9.81;
    equation
      Fx = Thrust - Forces_w[2] * cos(alpha) * cos(beta) - Forces_w[3] * cos(alpha) * sin(beta) + Forces_w[1] * sin(alpha) - m * g * sin(EulerAngles[2]);
      Fy = (-Forces_w[2] * sin(beta)) + Forces_w[3] * cos(beta) + m * g * sin(EulerAngles[1]) * cos(EulerAngles[2]);
      Fz = (-Forces_w[2] * sin(alpha) * cos(beta)) - Forces_w[3] * sin(alpha) * sin(beta) - Forces_w[1] * cos(alpha) + m * g * cos(EulerAngles[1]) * cos(EulerAngles[2]);
      Forces_b = {Fx, Fy, Fz};
    end ForceInBodyAxis;

    block Mux6
      extends Modelica.Blocks.Icons.Block;
      Modelica.Blocks.Interfaces.RealInput u1 annotation(
        Placement(visible = true, transformation(origin = {-120, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u2 annotation(
        Placement(visible = true, transformation(origin = {-108, 46}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u3 annotation(
        Placement(visible = true, transformation(origin = {-106, 16}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u4 annotation(
        Placement(visible = true, transformation(origin = {-106, -14}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u5 annotation(
        Placement(visible = true, transformation(origin = {-106, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u6 annotation(
        Placement(visible = true, transformation(origin = {-104, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y[6] annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      y = {u1, u2, u3, u4, u5, u6};
    end Mux6;

    class PILBlocks
      extends Modelica.Icons.InterfacesPackage;

      block SensorPacketGen
        parameter Real lat_init(final unit = "deg") = 19.1346445 "Initial Latitude";
        parameter Real lon_init(final unit = "deg") = 72.9121639 "Initial Longitude";
        Modelica.Blocks.Interfaces.RealInput Xe[3] annotation(
          Placement(visible = true, transformation(origin = {-104, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vb[3] annotation(
          Placement(visible = true, transformation(origin = {-104, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput omega_b[3] annotation(
          Placement(visible = true, transformation(origin = {-106, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput EulerAngles[3] annotation(
          Placement(visible = true, transformation(origin = {-106, -6}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager1(enableExternalTrigger = false, useBackwardPropagatedBufferSize = false, useBackwardSampleTimePropagation = true) annotation(
          Placement(visible = true, transformation(origin = {-54, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        aerospace.Components.IncidenceSideslipandAirspeed incidenceSideslipandAirspeed1 annotation(
          Placement(visible = true, transformation(origin = {-86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Integer time_epoch(start = 0) "Time in usec since epoch";
        Real lat_data(start = 0.0) "Latitude in degrees";
        Real dlat(start = 0.0);
        Real dlong(start = 0.0);
        Real lon_data(start = 0.0) "Longitude in degrees";
        Real agl(start = 0.0);
        Real cog_deg(start = 0.0) "Course over Ground in Degrees";
        Real cog(start = 0.0);
        Real grnd_speed(start = 0.0) "Ground Speed";
        Real delx(start = 0.0);
        Real dely(start = 0.0);
        Real Ax(start = 0.0);
        Real Ay(start = 0.0);
        Real Az(start = 0.0);
        Real p(start = 0.0);
        Real q(start = 0.0);
        Real r(start = 0.0);
        Real phi(start = 0.0);
        Real theta(start = 0.0);
        Real psi(start = 0.0);
        Real alpha(start = 0.0);
        Real beta(start = 0.0);
        Real vcas(start = 0.0) "Calibrated Airspeed";
        Real asl(start = 0.0);
        Real static_pressure(start = 0.0);
        Real chk_sum(start = 0.0);
        Integer HeaderValue(start = 0);
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger EpochTime(bitOffset = 0, nu = 1, width = 64) annotation(
          Placement(visible = true, transformation(origin = {-54, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat latitude(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat longitude(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat AGL(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat COG_DEG(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_x(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_y(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_z(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat P(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Q(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat R(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Phi(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Theta(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Psi(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Alpha(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Beta(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat VCAS(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat ASL(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {42, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Static_Pressure(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {42, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger CheckSum(nu = 1, width = 8) annotation(
          Placement(visible = true, transformation(origin = {42, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger Header(bitOffset = 0, nu = 1, width = 8) annotation(
          Placement(visible = true, transformation(origin = {-54, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat GroundSpeed(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Communication.SerialPortSend serialPortSend1(autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, parity = 0, sampleTime = 0.001, startTime = 0, userBufferSize = 86) annotation(
          Placement(visible = true, transformation(origin = {64, -24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        connect(COG_DEG.pkgOut[1], GroundSpeed.pkgIn);
        connect(GroundSpeed.pkgOut[1], A_x.pkgIn) annotation(
          Line(points = {{-20, 74}, {-20, 74}, {-20, 68}, {-20, 68}}));
        connect(grnd_speed, GroundSpeed.u[1]) annotation(
          Line);
        connect(COG_DEG.pkgIn, AGL.pkgOut[1]) annotation(
          Line(points = {{-54, -62}, {-54, -62}, {-54, -56}, {-54, -56}}));
        connect(cog_deg, COG_DEG.u[1]) annotation(
          Line);
        connect(R.pkgIn, Q.pkgOut[1]) annotation(
          Line(points = {{-20, -61.2}, {-20, -57.2}}));
        connect(R.pkgOut[1], Phi.pkgIn) annotation(
          Line(points = {{-20, -83}, {-20, -90}, {-6, -90}, {-6, 98}, {14, 98}, {14, 95}}));
        connect(Q.pkgIn, P.pkgOut[1]) annotation(
          Line(points = {{-20, -35.2}, {-20, -31.2}}));
        connect(P.pkgIn, A_z.pkgOut[1]) annotation(
          Line(points = {{-20, -9.2}, {-20, -5.2}}));
        connect(A_z.pkgIn, A_y.pkgOut[1]) annotation(
          Line(points = {{-20, 16.8}, {-20, 21.8}}));
        connect(A_y.pkgIn, A_x.pkgOut[1]) annotation(
          Line(points = {{-20, 42.8}, {-20, 46.8}}));
        connect(Header.pkgIn, packager1.pkgOut) annotation(
          Line(points = {{-54, 68}, {-54, 68}, {-54, 74}, {-54, 74}}));
        connect(EpochTime.pkgIn, Header.pkgOut[1]) annotation(
          Line(points = {{-54, 42}, {-54, 42}, {-54, 48}, {-54, 48}}));
        connect(longitude.pkgOut[1], AGL.pkgIn) annotation(
          Line(points = {{-54, -30.8}, {-54, -30.8}, {-54, -30.8}, {-54, -30.8}, {-54, -36.8}, {-54, -36.8}, {-54, -36.8}, {-54, -36.8}}));
        connect(latitude.pkgOut[1], longitude.pkgIn) annotation(
          Line(points = {{-54, -4.8}, {-54, -9.8}}));
        connect(EpochTime.pkgOut[1], latitude.pkgIn) annotation(
          Line(points = {{-54, 21.2}, {-54, 16.2}}));
        connect(Static_Pressure.pkgOut[1], CheckSum.pkgIn) annotation(
          Line(points = {{42, 48}, {42, 48}, {42, 40}, {42, 40}}));
        connect(Static_Pressure.pkgIn, ASL.pkgOut[1]) annotation(
          Line(points = {{42, 68}, {42, 68}, {42, 74}, {42, 74}}));
        connect(VCAS.pkgOut[1], ASL.pkgIn) annotation(
          Line(points = {{14, -56}, {14, -56}, {14, -60}, {26, -60}, {26, 98}, {42, 98}, {42, 94}, {42, 94}}));
        connect(VCAS.pkgIn, Beta.pkgOut[1]) annotation(
          Line(points = {{14, -36}, {14, -36}, {14, -30}, {14, -30}}));
        connect(Beta.pkgIn, Alpha.pkgOut[1]) annotation(
          Line(points = {{14, -10}, {14, -10}, {14, -4}, {14, -4}}));
        connect(Alpha.pkgIn, Psi.pkgOut[1]) annotation(
          Line(points = {{14, 17}, {14, 21}}));
        connect(Psi.pkgIn, Theta.pkgOut[1]) annotation(
          Line(points = {{14, 43}, {14, 48}}));
        connect(Theta.pkgIn, Phi.pkgOut[1]) annotation(
          Line(points = {{14, 69}, {14, 74}}));
        connect(vcas, incidenceSideslipandAirspeed1.Airspeed) annotation(
          Line);
        connect(beta, incidenceSideslipandAirspeed1.beta) annotation(
          Line);
        connect(alpha, incidenceSideslipandAirspeed1.alpha) annotation(
          Line);
        connect(Vb, incidenceSideslipandAirspeed1.Vb) annotation(
          Line);
/* Header Calculation */
        if mod(time, 1.0) == 0 then
          HeaderValue = 127;
        else
          HeaderValue = 126;
        end if;
/* Latutude and Longitude Calculation 
           111319.49 = (Radius of earth*pi)/180 */
        dlat = Xe[1] / 111319.49;
        dlong = Xe[2] / 111319.49;
        lat_data = lat_init + dlat;
        lon_data = lon_init + dlong;
/* COG */
        delx = Xe[1] - pre(Xe[1]);
        dely = Xe[2] - pre(Xe[2]);
        cog = atan2(dely, delx) * 57.295779513;
        if cog < 0 then
          cog_deg = cog + 360;
        else
          cog_deg = cog;
        end if;
/* Ground Speed */
        grnd_speed = vcas * cos(theta);
/* Angular Rates */
        p = omega_b[1];
        q = omega_b[2];
        r = omega_b[3];
/* Euler Angles */
        phi = EulerAngles[1];
        theta = EulerAngles[2];
        psi = EulerAngles[3];
/* Altitudes */
        agl = -Xe[3];
        asl = -Xe[3];
/* Static Pressure 
        P = P0*((1-0.0065(h/T0))^5.2561)
        P0 = 101325 Nm-3;T0 = 288.15 K(15 degC);
        */
        static_pressure = 101325 * (1 - 0.0065 * (agl / 288.15)) ^ 5.2561;
/* usec since Epoch */
        time_epoch = aerospace.Components.PILBlocks.get_time();
/* Accelerations */
        Ax = der(Vb[1]);
        Ay = der(Vb[2]);
        Az = der(Vb[3]);
        connect(HeaderValue, Header.u);
        connect(time_epoch, EpochTime.u);
        connect(lat_data, latitude.u[1]);
        connect(lon_data, longitude.u[1]);
        connect(p, P.u[1]);
        connect(q, Q.u[1]);
        connect(r, R.u[1]);
        connect(phi, Phi.u[1]);
        connect(theta, Theta.u[1]);
        connect(psi, Psi.u[1]);
        connect(alpha, Alpha.u[1]);
        connect(beta, Beta.u[1]);
        connect(vcas, VCAS.u[1]);
        connect(asl, ASL.u[1]);
        connect(agl, AGL.u[1]);
        connect(static_pressure, Static_Pressure.u[1]);
        connect(serialPortSend1.pkgIn, CheckSum.pkgOut[1]);
      end SensorPacketGen;

      block SerialCommunication
        import Time_F = FCSys.Utilities.Time;
        parameter Real lat_init(final unit = "deg") = 19.1346445 "Initial Latitude";
        parameter Real lon_init(final unit = "deg") = 72.9121639 "Initial Longitude";
        Modelica.Blocks.Interfaces.RealInput Xe[3] annotation(
          Placement(visible = true, transformation(origin = {-104, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vb[3] annotation(
          Placement(visible = true, transformation(origin = {-104, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput omega_b[3] annotation(
          Placement(visible = true, transformation(origin = {-106, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput EulerAngles[3] annotation(
          Placement(visible = true, transformation(origin = {-106, -6}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager1(enableExternalTrigger = false, useBackwardPropagatedBufferSize = false, useBackwardSampleTimePropagation = true) annotation(
          Placement(visible = true, transformation(origin = {-54, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        aerospace.Components.IncidenceSideslipandAirspeed incidenceSideslipandAirspeed1 annotation(
          Placement(visible = true, transformation(origin = {-86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Integer time_epoch(start = 0) "Time in usec since epoch";
        Real lat_data(start = 0.0) "Latitude in degrees";
        Real dlat(start = 0.0);
        Real dlong(start = 0.0);
        Real lon_data(start = 0.0) "Longitude in degrees";
        Real agl(start = 0.0);
        Real cog_deg(start = 0.0) "Course over Ground in Degrees";
        Real cog(start = 0.0);
        Real grnd_speed(start = 0.0) "Ground Speed";
        Real delx(start = 0.0);
        Real dely(start = 0.0);
        Real Ax(start = 0.0);
        Real Ay(start = 0.0);
        Real Az(start = 0.0);
        Real p(start = 0.0);
        Real q(start = 0.0);
        Real r(start = 0.0);
        Real phi(start = 0.0);
        Real theta(start = 0.0);
        Real psi(start = 0.0);
        Real alpha(start = 0.0);
        Real beta(start = 0.0);
        Real vcas(start = 0.0) "Calibrated Airspeed";
        Real asl(start = 0.0);
        Real static_pressure(start = 0.0);
        Real chk_sum(start = 0.0);
        Integer HeaderValue(start = 0);
        Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialPortReceive1(Serial_Port = "COM1", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, parity = 0, sampleTime = 0.1, startTime = 0, userBufferSize = 16 * 64) annotation(
          Placement(visible = true, transformation(origin = {78, 78}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetFloat Aileron_Data(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {78, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetFloat Elevator_Data(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {78, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetFloat Throttle_Data(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {78, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetFloat Rudder_Data(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {78, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Aileron annotation(
          Placement(visible = true, transformation(origin = {108, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Elevator annotation(
          Placement(visible = true, transformation(origin = {110, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Throttle annotation(
          Placement(visible = true, transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Rudder annotation(
          Placement(visible = true, transformation(origin = {108, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger EpochTime(bitOffset = 0, nu = 1, width = 64) annotation(
          Placement(visible = true, transformation(origin = {-54, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat latitude(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat longitude(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat AGL(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat COG_DEG(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-54, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_x(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_y(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_z(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat P(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Q(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat R(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Phi(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Theta(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Psi(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Alpha(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Beta(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat VCAS(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {14, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat ASL(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {42, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Static_Pressure(nu = 1) annotation(
          Placement(visible = true, transformation(origin = {42, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger CheckSum(nu = 1, width = 8) annotation(
          Placement(visible = true, transformation(origin = {42, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger Header(bitOffset = 0, nu = 1, width = 8) annotation(
          Placement(visible = true, transformation(origin = {-54, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat GroundSpeed(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
          Placement(visible = true, transformation(origin = {-20, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica_DeviceDrivers.Blocks.Communication.SerialPortSend serialPortSend1(autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, parity = 0, sampleTime = 0.001, startTime = 0, userBufferSize = 86) annotation(
          Placement(visible = true, transformation(origin = {42, -2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger HeaderIn(bitOffset = 0, nu = 1, width = 8) annotation(
          Placement(visible = true, transformation(origin = {78, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(Aileron_Data.pkgIn, HeaderIn.pkgOut[1]) annotation(
          Line(points = {{78, 26}, {78, 26}, {78, 36}, {78, 36}}));
        connect(HeaderIn.pkgIn, serialPortReceive1.pkgOut) annotation(
          Line(points = {{78, 56}, {78, 56}, {78, 68}, {78, 68}}));
        connect(Aileron_Data.y[1], Aileron) annotation(
          Line(points = {{89, 16}, {108, 16}}, color = {0, 0, 127}));
        connect(Elevator_Data.pkgIn, Aileron_Data.pkgOut[1]) annotation(
          Line(points = {{78, -3}, {78, 5}}));
        connect(Elevator_Data.y[1], Elevator) annotation(
          Line(points = {{89, -14}, {110, -14}}, color = {0, 0, 127}));
        connect(Throttle_Data.pkgIn, Elevator_Data.pkgOut[1]) annotation(
          Line(points = {{78, -39}, {78, -25}}));
        connect(Rudder_Data.pkgIn, Throttle_Data.pkgOut[1]) annotation(
          Line(points = {{78, -63}, {78, -61}}));
        connect(Throttle_Data.y[1], Throttle) annotation(
          Line(points = {{89, -50}, {108, -50}}, color = {0, 0, 127}));
        connect(serialPortSend1.pkgIn, CheckSum.pkgOut[1]) annotation(
          Line);
        connect(COG_DEG.pkgOut[1], GroundSpeed.pkgIn);
        connect(GroundSpeed.pkgOut[1], A_x.pkgIn) annotation(
          Line(points = {{-20, 74}, {-20, 74}, {-20, 68}, {-20, 68}}));
        connect(grnd_speed, GroundSpeed.u[1]) annotation(
          Line);
        connect(COG_DEG.pkgIn, AGL.pkgOut[1]) annotation(
          Line(points = {{-54, -62}, {-54, -62}, {-54, -56}, {-54, -56}}));
        connect(cog_deg, COG_DEG.u[1]) annotation(
          Line);
        connect(R.pkgIn, Q.pkgOut[1]) annotation(
          Line(points = {{-20, -61.2}, {-20, -57.2}}));
        connect(R.pkgOut[1], Phi.pkgIn) annotation(
          Line(points = {{-20, -83}, {-20, -90}, {-6, -90}, {-6, 98}, {14, 98}, {14, 95}}));
        connect(Q.pkgIn, P.pkgOut[1]) annotation(
          Line(points = {{-20, -35.2}, {-20, -31.2}}));
        connect(P.pkgIn, A_z.pkgOut[1]) annotation(
          Line(points = {{-20, -9.2}, {-20, -5.2}}));
        connect(A_z.pkgIn, A_y.pkgOut[1]) annotation(
          Line(points = {{-20, 16.8}, {-20, 21.8}}));
        connect(A_y.pkgIn, A_x.pkgOut[1]) annotation(
          Line(points = {{-20, 42.8}, {-20, 46.8}}));
        connect(Header.pkgIn, packager1.pkgOut) annotation(
          Line(points = {{-54, 68}, {-54, 68}, {-54, 74}, {-54, 74}}));
        connect(EpochTime.pkgIn, Header.pkgOut[1]) annotation(
          Line(points = {{-54, 42}, {-54, 42}, {-54, 48}, {-54, 48}}));
        connect(longitude.pkgOut[1], AGL.pkgIn) annotation(
          Line(points = {{-54, -30.8}, {-54, -30.8}, {-54, -30.8}, {-54, -30.8}, {-54, -36.8}, {-54, -36.8}, {-54, -36.8}, {-54, -36.8}}));
        connect(latitude.pkgOut[1], longitude.pkgIn) annotation(
          Line(points = {{-54, -4.8}, {-54, -9.8}}));
        connect(EpochTime.pkgOut[1], latitude.pkgIn) annotation(
          Line(points = {{-54, 21.2}, {-54, 16.2}}));
        connect(Static_Pressure.pkgOut[1], CheckSum.pkgIn) annotation(
          Line(points = {{42, 48}, {42, 48}, {42, 40}, {42, 40}}));
        connect(Static_Pressure.pkgIn, ASL.pkgOut[1]) annotation(
          Line(points = {{42, 68}, {42, 68}, {42, 74}, {42, 74}}));
        connect(VCAS.pkgOut[1], ASL.pkgIn) annotation(
          Line(points = {{14, -56}, {14, -56}, {14, -60}, {26, -60}, {26, 98}, {42, 98}, {42, 94}, {42, 94}}));
        connect(VCAS.pkgIn, Beta.pkgOut[1]) annotation(
          Line(points = {{14, -36}, {14, -36}, {14, -30}, {14, -30}}));
        connect(Beta.pkgIn, Alpha.pkgOut[1]) annotation(
          Line(points = {{14, -10}, {14, -10}, {14, -4}, {14, -4}}));
        connect(Alpha.pkgIn, Psi.pkgOut[1]) annotation(
          Line(points = {{14, 17}, {14, 21}}));
        connect(Psi.pkgIn, Theta.pkgOut[1]) annotation(
          Line(points = {{14, 43}, {14, 48}}));
        connect(Theta.pkgIn, Phi.pkgOut[1]) annotation(
          Line(points = {{14, 69}, {14, 74}}));
        connect(vcas, incidenceSideslipandAirspeed1.Airspeed) annotation(
          Line);
        connect(beta, incidenceSideslipandAirspeed1.beta) annotation(
          Line);
        connect(alpha, incidenceSideslipandAirspeed1.alpha) annotation(
          Line);
        connect(Vb, incidenceSideslipandAirspeed1.Vb) annotation(
          Line);
/* Header Calculation */
        if mod(time, 1.0) == 0 then
          HeaderValue = 127;
        else
          HeaderValue = 126;
        end if;
/* Latutude and Longitude Calculation 
           111319.49 = (Radius of earth*pi)/180 */
        dlat = Xe[1] / 111319.49;
        dlong = Xe[2] / 111319.49;
        lat_data = lat_init + dlat;
        lon_data = lon_init + dlong;
/* COG */
        delx = Xe[1] - pre(Xe[1]);
        dely = Xe[2] - pre(Xe[2]);
        cog = atan2(dely, delx) * 57.295779513;
        if cog < 0 then
          cog_deg = cog + 360;
        else
          cog_deg = cog;
        end if;
/* Ground Speed */
        grnd_speed = vcas * cos(theta);
/* Angular Rates */
        p = omega_b[1];
        q = omega_b[2];
        r = omega_b[3];
/* Euler Angles */
        phi = EulerAngles[1];
        theta = EulerAngles[2];
        psi = EulerAngles[3];
/* Altitudes */
        agl = -Xe[3];
        asl = -Xe[3];
/* Static Pressure 
        P = P0*((1-0.0065(h/T0))^5.2561)
        P0 = 101325 Nm-3;T0 = 288.15 K(15 degC);
        */
        static_pressure = 101325 * (1 - 0.0065 * (agl / 288.15)) ^ 5.2561;
/* usec since Epoch (Requires FCSys Package. Download from https://github.com/kdavies4/FCSys/archive/v0.2.6.zip ) */
        time_epoch = Time_F.get_time();
/* Accelerations */
        Ax = der(Vb[1]);
        Ay = der(Vb[2]);
        Az = der(Vb[3]);
        connect(HeaderValue, Header.u);
        connect(time_epoch, EpochTime.u);
        connect(lat_data, latitude.u[1]);
        connect(lon_data, longitude.u[1]);
        connect(p, P.u[1]);
        connect(q, Q.u[1]);
        connect(r, R.u[1]);
        connect(phi, Phi.u[1]);
        connect(theta, Theta.u[1]);
        connect(psi, Psi.u[1]);
        connect(alpha, Alpha.u[1]);
        connect(beta, Beta.u[1]);
        connect(vcas, VCAS.u[1]);
        connect(asl, ASL.u[1]);
        connect(agl, AGL.u[1]);
        connect(static_pressure, Static_Pressure.u[1]);
        connect(Rudder_Data.y[1], Rudder) annotation(
          Line(points = {{90, -74}, {100, -74}, {100, -74}, {108, -74}}, color = {0, 0, 127}));
      end SerialCommunication;

      function get_time
        extends Modelica.Icons.Function;
        output Integer t "Time in milliseconds since January 1, 1970";
      
        external "C" ;
        annotation(
          Include = "#include \"time.c\"");
      end get_time;
    end PILBlocks;

    block fmstimestamp
      Real dummy(start = 0);
      Real clock "time in ms";
      Modelica.Blocks.Interfaces.RealOutput fms_ts_out annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      dummy = 0;
      clock = Modelica_DeviceDrivers.OperatingSystem.clock(dummy);
      connect(clock, fms_ts_out);
    end fmstimestamp;
    annotation(
      Icon(coordinateSystem(grid = {0.5, 0.5})));
  end Components;

  class Examples
    extends Modelica.Icons.ExamplesPackage;

block CessnaAircraft
  extends Modelica.Blocks.Icons.Block;
  aerospace.Components.SixDOFEoM sixDOFEoM1(Euler_init = {0.0, 0.011191045110034, 0.0}, I = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}}, m = 1043.26, omega_init = {0.0, 0.0, 0.0}, vb_init = {60.0, 0.0, 0.0}, xe_init = {0.0, 0.0, -100}) annotation(
    Placement(visible = true, transformation(origin = {20, 58}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  aerospace.Components.AerodynamicForcesandMoments aerodynamicForcesandMoments1 annotation(
    Placement(visible = true, transformation(origin = {-58, 16}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  aerospace.Components.IncidenceSideslipandAirspeed incidenceSideslipandAirspeed1 annotation(
    Placement(visible = true, transformation(origin = {89, 9}, extent = {{9, -9}, {-9, 9}}, rotation = 90)));
  aerospace.Components.DynamicPressure dynamicPressure1 annotation(
    Placement(visible = true, transformation(origin = {30, -2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  aerospace.Components.AerodynamicCoefficients aerodynamicCoefficients1 annotation(
    Placement(visible = true, transformation(origin = {32, -40}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant AirDensity(k = 1.225) annotation(
    Placement(visible = true, transformation(origin = {53, 15}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  aerospace.Components.ForceInBodyAxis forceInBodyAxis1 annotation(
    Placement(visible = true, transformation(origin = {-83, 53}, extent = {{9, -9}, {-9, 9}}, rotation = -90)));
  aerospace.Components.Mux6 mux61 annotation(
    Placement(visible = true, transformation(origin = {-23, -39}, extent = {{21, -21}, {-21, 21}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Thrust(final quantity = "Force", final unit = "N") annotation(
    Placement(visible = true, transformation(origin = {-116, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-103, 59}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Elevator(quantity = "Angle", final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {-120, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-103, 21}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Aileron(quantity = "Angle", final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {-118, -8}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-103, -19}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Rudder(quantity = "Angle", final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {-118, -46}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-103, -59}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Vb[3](each final quantity = "Velocity", final unit = "m/s") annotation(
    Placement(visible = true, transformation(origin = {108, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput EulerAngles[3](each final quantity = "Angle", final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {128, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput omega_b[3](each final quantity = "AngularVelocity", final unit = "rad/s") annotation(
    Placement(visible = true, transformation(origin = {132, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput DCMbe[3, 3](each unit = "1") annotation(
    Placement(visible = true, transformation(origin = {110, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  /*aerospace.Components.fmstimestamp fmstimestamp1 annotation(
                                                                                                                                                                                  Placement(visible = true, transformation(origin = {34, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                                                                                                                                                                                Modelica.Blocks.Sources.Clock fmsclock(offset = 0, startTime = 0) annotation(
                                                                                                                                                                                  Placement(visible = true, transformation(origin = {-58, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                                                                                                                                                                                Modelica.Blocks.Interfaces.RealOutput fmsclkout annotation(
                                                                                                                                                                                  Placement(visible = true, transformation(origin = {104, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                                                                                                                                                                                Modelica.Blocks.Interfaces.BooleanOutput fmsclksqwave annotation(
                                                                                                                                                                                  Placement(visible = true, transformation(origin = {104, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                                                                                                                                                                                Boolean clkoutpulse(start = false);*/
  Modelica.Blocks.Interfaces.RealOutput Xe[3] annotation(
    Placement(visible = true, transformation(origin = {132, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(Xe, sixDOFEoM1.Xe);
/*
  if mod(fmsclock.y * 1000.0, 2.0) == 0.0 then
    clkoutpulse = not pre(clkoutpulse);
  else
    clkoutpulse = pre(clkoutpulse);
  end if;
  connect(clkoutpulse, fmsclksqwave);
  connect(fmsclock.y, fmsclkout) annotation(
    Line);
  */
  connect(Rudder, aerodynamicCoefficients1.Rudder) annotation(
    Line);
  connect(aerodynamicCoefficients1.Rudder, Rudder) annotation(
    Line);
  connect(aerodynamicCoefficients1.Cn, mux61.u6) annotation(
    Line(points = {{10, -54}, {0, -54}}, color = {0, 0, 127}));
  connect(aerodynamicCoefficients1.Cm, mux61.u5) annotation(
    Line(points = {{10, -50}, {0, -50}, {0, -49.5}}, color = {0, 0, 127}));
  connect(aerodynamicCoefficients1.Cl, mux61.u4) annotation(
    Line(points = {{10, -46}, {2, -46}, {2, -45}, {0, -45}}, color = {0, 0, 127}));
  connect(aerodynamicCoefficients1.CY, mux61.u3) annotation(
    Line(points = {{10, -34}, {5, -34}, {5, -33}, {0, -33}}, color = {0, 0, 127}));
  connect(aerodynamicCoefficients1.CD, mux61.u2) annotation(
    Line(points = {{10, -30}, {5, -30}, {5, -28.5}, {0, -28.5}}, color = {0, 0, 127}));
  connect(aerodynamicCoefficients1.CL, mux61.u1) annotation(
    Line(points = {{10, -26}, {5, -26}, {5, -24}, {0, -24}}, color = {0, 0, 127}));
  connect(forceInBodyAxis1.Thrust, Thrust);
  connect(forceInBodyAxis1.EulerAngles, sixDOFEoM1.EulerAngles);
  connect(forceInBodyAxis1.beta, incidenceSideslipandAirspeed1.beta);
  connect(forceInBodyAxis1.alpha, incidenceSideslipandAirspeed1.alpha);
  connect(forceInBodyAxis1.Forces_w, aerodynamicForcesandMoments1.Forces);
  connect(forceInBodyAxis1.Forces_b, sixDOFEoM1.Forces);
  connect(aerodynamicForcesandMoments1.qbar, dynamicPressure1.qbar);
  connect(aerodynamicForcesandMoments1.Coefficients, mux61.y);
  connect(aerodynamicForcesandMoments1.Moments, sixDOFEoM1.Moments);
  connect(dynamicPressure1.Vb, sixDOFEoM1.Vb);
  connect(dynamicPressure1.rho, AirDensity.y);
  connect(aerodynamicCoefficients1.alpha, incidenceSideslipandAirspeed1.alpha);
  connect(aerodynamicCoefficients1.beta, incidenceSideslipandAirspeed1.beta);
  connect(aerodynamicCoefficients1.Airspeed, incidenceSideslipandAirspeed1.Airspeed);
  connect(aerodynamicCoefficients1.Elevator, Elevator);
  connect(aerodynamicCoefficients1.Aileron, Aileron);
  connect(aerodynamicCoefficients1.Rudder, Rudder);
  connect(aerodynamicCoefficients1.omega_b, sixDOFEoM1.omega_b);
  connect(incidenceSideslipandAirspeed1.Vb, sixDOFEoM1.Vb);
/* Inputs */
  connect(Thrust, forceInBodyAxis1.Thrust);
  connect(Aileron, aerodynamicCoefficients1.Aileron);
  connect(Elevator, aerodynamicCoefficients1.Elevator);
  connect(Rudder, aerodynamicCoefficients1.Rudder);
/* Outputs */
  connect(Vb, sixDOFEoM1.Vb);
  connect(EulerAngles, sixDOFEoM1.EulerAngles);
  connect(omega_b, sixDOFEoM1.omega_b);
  connect(DCMbe, sixDOFEoM1.DCMbe);
  annotation(
    Icon(coordinateSystem(grid = {0.5, 0.5})));
end CessnaAircraft;
    

    model CessnaWithTrim
      extends Modelica.Icons.Example;
      aerospace.Examples.CessnaAircraft cessnaAircraft1 annotation(
        Placement(visible = true, transformation(origin = {39, 19}, extent = {{-29, -29}, {29, 29}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Elevator(k = -0.0306583) annotation(
        Placement(visible = true, transformation(origin = {-84, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Aileron(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Rudder(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Thrust(k = 1516.042952) annotation(
        Placement(visible = true, transformation(origin = {-84, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(cessnaAircraft1.Rudder, Rudder.y) annotation(
        Line(points = {{10, 2}, {-20, 2}, {-20, -30}, {-73, -30}}, color = {0, 0, 127}));
      connect(Thrust.y, cessnaAircraft1.Thrust) annotation(
        Line(points = {{-73, 72}, {-20, 72}, {-20, 36}, {10, 36}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Elevator, Elevator.y) annotation(
        Line(points = {{10, 26}, {-40, 26}, {-40, 38}, {-73, 38}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Aileron, Aileron.y) annotation(
        Line(points = {{10, 14}, {-40, 14}, {-40, 0}, {-73, 0}}, color = {0, 0, 127}));
    end CessnaWithTrim;

    model CessnaWithTrimRealTime
      extends Modelica.Icons.Example;
      aerospace.Examples.CessnaAircraft cessnaAircraft1 annotation(
        Placement(visible = true, transformation(origin = {39, 19}, extent = {{-29, -29}, {29, 29}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Elevator(k = -0.1745) annotation(
        Placement(visible = true, transformation(origin = {-84, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Aileron(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Rudder(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Thrust(k = 1830.5) annotation(
        Placement(visible = true, transformation(origin = {-84, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {82, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.HardwareIO.Comedi.ComediConfig comedi annotation(
        Placement(visible = true, transformation(origin = {-80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //Modelica_DeviceDrivers.Blocks.HardwareIO.Comedi.DIOWrite dioWrite annotation(
      Modelica_DeviceDrivers.Blocks.HardwareIO.Comedi.DIOWrite dioWrite(comedi = comedi.dh) annotation(
        Placement(visible = true, transformation(origin = {80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Boolean dio_out;
    equation
      when sample(0, 0.01) then
        dio_out = not pre(dio_out);
      end when;
      connect(cessnaAircraft1.Rudder, Rudder.y) annotation(
        Line(points = {{10, 2}, {-20, 2}, {-20, -30}, {-73, -30}}, color = {0, 0, 127}));
      connect(Thrust.y, cessnaAircraft1.Thrust) annotation(
        Line(points = {{-73, 72}, {-20, 72}, {-20, 36}, {10, 36}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Elevator, Elevator.y) annotation(
        Line(points = {{10, 26}, {-40, 26}, {-40, 38}, {-73, 38}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Aileron, Aileron.y) annotation(
        Line(points = {{10, 14}, {-40, 14}, {-40, 0}, {-73, 0}}, color = {0, 0, 127}));
      connect(dio_out, dioWrite.u);
    end CessnaWithTrimRealTime;

    model CessnaWithTrimRealTimeWin
      extends Modelica.Icons.Example;
      extends Modelica.Icons.Example;
      aerospace.Examples.CessnaAircraft cessnaAircraft1 annotation(
        Placement(visible = true, transformation(origin = {39, 19}, extent = {{-29, -29}, {29, 29}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Elevator(k = -0.1745) annotation(
        Placement(visible = true, transformation(origin = {-84, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Aileron(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Rudder(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Thrust(k = 1830.5) annotation(
        Placement(visible = true, transformation(origin = {-84, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {72, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(cessnaAircraft1.Rudder, Rudder.y) annotation(
        Line(points = {{10, 2}, {-20, 2}, {-20, -30}, {-73, -30}}, color = {0, 0, 127}));
      connect(Thrust.y, cessnaAircraft1.Thrust) annotation(
        Line(points = {{-73, 72}, {-20, 72}, {-20, 36}, {10, 36}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Elevator, Elevator.y) annotation(
        Line(points = {{10, 26}, {-40, 26}, {-40, 38}, {-73, 38}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Aileron, Aileron.y) annotation(
        Line(points = {{10, 14}, {-40, 14}, {-40, 0}, {-73, 0}}, color = {0, 0, 127}));
    end CessnaWithTrimRealTimeWin;
    annotation(
      Icon(coordinateSystem(grid = {0.5, 0.5})));
  end Examples;

  model pils_test1
    model fms
      aerospace.Examples.CessnaAircraft cessnaAircraft1 annotation(
        Placement(visible = true, transformation(origin = {39, 19}, extent = {{-29, -29}, {29, 29}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Elevator(k = -0.1745) annotation(
        Placement(visible = true, transformation(origin = {-84, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Aileron(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Rudder(k = 0.0) annotation(
        Placement(visible = true, transformation(origin = {-84, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant Thrust(k = 1830.5) annotation(
        Placement(visible = true, transformation(origin = {-84, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
        Placement(visible = true, transformation(origin = {82, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //Modelica_DeviceDrivers.Blocks.HardwareIO.Comedi.ComediConfig comedi annotation(
      //   Placement(visible = true, transformation(origin = {-80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //Modelica_DeviceDrivers.Blocks.HardwareIO.Comedi.DIOWrite dioWrite(comedi = comedi.dh)  annotation(
      //    Placement(visible = true, transformation(origin = {80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //Boolean dio_out;
      aerospace.pils_test1.serialhandling serialhandling1 annotation(
        Placement(visible = true, transformation(origin = {44, -58}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
    equation
//when sample(0,0.001) then
// dio_out = not pre(dio_out);
//end when;
      connect(cessnaAircraft1.Rudder, Rudder.y) annotation(
        Line(points = {{10, 2}, {-20, 2}, {-20, -30}, {-73, -30}}, color = {0, 0, 127}));
      connect(Thrust.y, cessnaAircraft1.Thrust) annotation(
        Line(points = {{-73, 72}, {-20, 72}, {-20, 36}, {10, 36}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Elevator, Elevator.y) annotation(
        Line(points = {{10, 26}, {-40, 26}, {-40, 38}, {-73, 38}}, color = {0, 0, 127}));
      connect(cessnaAircraft1.Aileron, Aileron.y) annotation(
        Line(points = {{10, 14}, {-40, 14}, {-40, 0}, {-73, 0}}, color = {0, 0, 127}));
//  connect(dio_out,dioWrite.u);
      connect(serialhandling1.Vb, cessnaAircraft1.Vb);
      connect(serialhandling1.EulerAngles, cessnaAircraft1.EulerAngles);
      connect(serialhandling1.omega_b, cessnaAircraft1.omega_b);
      connect(serialhandling1.Xe, cessnaAircraft1.Xe);
      annotation(
        __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "rungekutta", lv = "LOG_STATS"));
    end fms;

    model serialhandling
      import Time_F = FCSys.Utilities.Time;
      parameter Real lat_init(final unit = "deg") = 19.1346445 "Initial Latitude";
      parameter Real lon_init(final unit = "deg") = 72.9121639 "Initial Longitude";
      Modelica.Blocks.Interfaces.RealInput Xe[3] annotation(
        Placement(visible = true, transformation(origin = {-104, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vb[3] annotation(
        Placement(visible = true, transformation(origin = {-104, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput omega_b[3] annotation(
        Placement(visible = true, transformation(origin = {-106, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput EulerAngles[3] annotation(
        Placement(visible = true, transformation(origin = {-106, -6}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager1(enableExternalTrigger = false, useBackwardPropagatedBufferSize = false, useBackwardSampleTimePropagation = true) annotation(
        Placement(visible = true, transformation(origin = {-54, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      aerospace.Components.IncidenceSideslipandAirspeed incidenceSideslipandAirspeed1 annotation(
        Placement(visible = true, transformation(origin = {-86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Integer time_epoch(start = 0) "Time in usec since epoch";
      Real lat_data(start = 0.0) "Latitude in degrees";
      Real dlat(start = 0.0);
      Real dlong(start = 0.0);
      Real lon_data(start = 0.0) "Longitude in degrees";
      Real agl(start = 0.0);
      Real cog_deg(start = 0.0) "Course over Ground in Degrees";
      Real cog(start = 0.0);
      Real grnd_speed(start = 0.0) "Ground Speed";
      Real delx(start = 0.0);
      Real dely(start = 0.0);
      Real Ax(start = 0.0);
      Real Ay(start = 0.0);
      Real Az(start = 0.0);
      Real p(start = 0.0);
      Real q(start = 0.0);
      Real r(start = 0.0);
      Real phi(start = 0.0);
      Real theta(start = 0.0);
      Real psi(start = 0.0);
      Real alpha(start = 0.0);
      Real beta(start = 0.0);
      Real vcas(start = 0.0) "Calibrated Airspeed";
      Real asl(start = 0.0);
      Real static_pressure(start = 0.0);
      Real chk_sum(start = 0.0);
      Integer HeaderValue(start = 0);
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger EpochTime(bitOffset = 0, nu = 1, width = 64) annotation(
        Placement(visible = true, transformation(origin = {-54, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat latitude(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-54, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat longitude(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-54, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat AGL(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-54, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat COG_DEG(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-54, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_x(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_y(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat A_z(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat P(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Q(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat R(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Phi(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {14, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Theta(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {14, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Psi(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {14, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Alpha(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {14, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Beta(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {14, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat VCAS(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {14, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat ASL(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {42, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat Static_Pressure(nu = 1) annotation(
        Placement(visible = true, transformation(origin = {42, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger CheckSum(nu = 1, width = 8) annotation(
        Placement(visible = true, transformation(origin = {42, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.PackUnsignedInteger Header(bitOffset = 0, nu = 1, width = 8) annotation(
        Placement(visible = true, transformation(origin = {-54, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddFloat GroundSpeed(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1) annotation(
        Placement(visible = true, transformation(origin = {-20, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica_DeviceDrivers.Blocks.Communication.SerialPortSend serialPortSend1(autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, parity = 0, sampleTime = 0.001, startTime = 0, userBufferSize = 86) annotation(
        Placement(visible = true, transformation(origin = {64, -24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
      connect(COG_DEG.pkgOut[1], GroundSpeed.pkgIn);
      connect(GroundSpeed.pkgOut[1], A_x.pkgIn) annotation(
        Line(points = {{-20, 74}, {-20, 74}, {-20, 68}, {-20, 68}}));
      connect(grnd_speed, GroundSpeed.u[1]) annotation(
        Line);
      connect(COG_DEG.pkgIn, AGL.pkgOut[1]) annotation(
        Line(points = {{-54, -62}, {-54, -62}, {-54, -56}, {-54, -56}}));
      connect(cog_deg, COG_DEG.u[1]) annotation(
        Line);
      connect(R.pkgIn, Q.pkgOut[1]) annotation(
        Line(points = {{-20, -61.2}, {-20, -57.2}}));
      connect(R.pkgOut[1], Phi.pkgIn) annotation(
        Line(points = {{-20, -83}, {-20, -90}, {-6, -90}, {-6, 98}, {14, 98}, {14, 95}}));
      connect(Q.pkgIn, P.pkgOut[1]) annotation(
        Line(points = {{-20, -35.2}, {-20, -31.2}}));
      connect(P.pkgIn, A_z.pkgOut[1]) annotation(
        Line(points = {{-20, -9.2}, {-20, -5.2}}));
      connect(A_z.pkgIn, A_y.pkgOut[1]) annotation(
        Line(points = {{-20, 16.8}, {-20, 21.8}}));
      connect(A_y.pkgIn, A_x.pkgOut[1]) annotation(
        Line(points = {{-20, 42.8}, {-20, 46.8}}));
      connect(Header.pkgIn, packager1.pkgOut) annotation(
        Line(points = {{-54, 68}, {-54, 68}, {-54, 74}, {-54, 74}}));
      connect(EpochTime.pkgIn, Header.pkgOut[1]) annotation(
        Line(points = {{-54, 42}, {-54, 42}, {-54, 48}, {-54, 48}}));
      connect(longitude.pkgOut[1], AGL.pkgIn) annotation(
        Line(points = {{-54, -30.8}, {-54, -30.8}, {-54, -30.8}, {-54, -30.8}, {-54, -36.8}, {-54, -36.8}, {-54, -36.8}, {-54, -36.8}}));
      connect(latitude.pkgOut[1], longitude.pkgIn) annotation(
        Line(points = {{-54, -4.8}, {-54, -9.8}}));
      connect(EpochTime.pkgOut[1], latitude.pkgIn) annotation(
        Line(points = {{-54, 21.2}, {-54, 16.2}}));
      connect(Static_Pressure.pkgOut[1], CheckSum.pkgIn) annotation(
        Line(points = {{42, 48}, {42, 48}, {42, 40}, {42, 40}}));
      connect(Static_Pressure.pkgIn, ASL.pkgOut[1]) annotation(
        Line(points = {{42, 68}, {42, 68}, {42, 74}, {42, 74}}));
      connect(VCAS.pkgOut[1], ASL.pkgIn) annotation(
        Line(points = {{14, -56}, {14, -56}, {14, -60}, {26, -60}, {26, 98}, {42, 98}, {42, 94}, {42, 94}}));
      connect(VCAS.pkgIn, Beta.pkgOut[1]) annotation(
        Line(points = {{14, -36}, {14, -36}, {14, -30}, {14, -30}}));
      connect(Beta.pkgIn, Alpha.pkgOut[1]) annotation(
        Line(points = {{14, -10}, {14, -10}, {14, -4}, {14, -4}}));
      connect(Alpha.pkgIn, Psi.pkgOut[1]) annotation(
        Line(points = {{14, 17}, {14, 21}}));
      connect(Psi.pkgIn, Theta.pkgOut[1]) annotation(
        Line(points = {{14, 43}, {14, 48}}));
      connect(Theta.pkgIn, Phi.pkgOut[1]) annotation(
        Line(points = {{14, 69}, {14, 74}}));
      connect(vcas, incidenceSideslipandAirspeed1.Airspeed) annotation(
        Line);
      connect(beta, incidenceSideslipandAirspeed1.beta) annotation(
        Line);
      connect(alpha, incidenceSideslipandAirspeed1.alpha) annotation(
        Line);
      connect(Vb, incidenceSideslipandAirspeed1.Vb) annotation(
        Line);
/* Header Calculation */
      if mod(time, 1.0) == 0 then
        HeaderValue = 127;
      else
        HeaderValue = 126;
      end if;
/* Latutude and Longitude Calculation 
         111319.49 = (Radius of earth*pi)/180 */
      dlat = Xe[1] / 111319.49;
      dlong = Xe[2] / 111319.49;
      lat_data = lat_init + dlat;
      lon_data = lon_init + dlong;
/* COG */
      delx = Xe[1] - pre(Xe[1]);
      dely = Xe[2] - pre(Xe[2]);
      cog = atan2(dely, delx) * 57.295779513;
      if cog < 0 then
        cog_deg = cog + 360;
      else
        cog_deg = cog;
      end if;
/* Ground Speed */
      grnd_speed = vcas * cos(theta);
/* Angular Rates */
      p = omega_b[1];
      q = omega_b[2];
      r = omega_b[3];
/* Euler Angles */
      phi = EulerAngles[1];
      theta = EulerAngles[2];
      psi = EulerAngles[3];
/* Altitudes */
      agl = -Xe[3];
      asl = -Xe[3];
/* Static Pressure 
      P = P0*((1-0.0065(h/T0))^5.2561)
      P0 = 101325 Nm-3;T0 = 288.15 K(15 degC);
      */
      static_pressure = 101325 * (1 - 0.0065 * (agl / 288.15)) ^ 5.2561;
/* usec since Epoch (Requires FCSys Package. Download from https://github.com/kdavies4/FCSys/archive/v0.2.6.zip ) */
      time_epoch = Time_F.get_time();
/* Accelerations */
      Ax = der(Vb[1]);
      Ay = der(Vb[2]);
      Az = der(Vb[3]);
      connect(HeaderValue, Header.u);
      connect(time_epoch, EpochTime.u);
      connect(lat_data, latitude.u[1]);
      connect(lon_data, longitude.u[1]);
      connect(p, P.u[1]);
      connect(q, Q.u[1]);
      connect(r, R.u[1]);
      connect(phi, Phi.u[1]);
      connect(theta, Theta.u[1]);
      connect(psi, Psi.u[1]);
      connect(alpha, Alpha.u[1]);
      connect(beta, Beta.u[1]);
      connect(vcas, VCAS.u[1]);
      connect(asl, ASL.u[1]);
      connect(agl, AGL.u[1]);
      connect(static_pressure, Static_Pressure.u[1]);
      connect(serialPortSend1.pkgIn, CheckSum.pkgOut[1]);
    end serialhandling;
  end pils_test1;
  annotation(
    Icon(coordinateSystem(grid = {0.5, 0.5})),
    uses(Modelica(version = "3.2.2"), Modelica_DeviceDrivers(version = "1.4.4")));
end aerospace;
