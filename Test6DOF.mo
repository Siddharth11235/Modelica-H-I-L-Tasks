model Test6DOF
  Real F[3] = {1, 0, 0};
  Real M[3] = {0 , 0, 1};
  Flight6DOF flight6DOF1 annotation(
    Placement(visible = true, transformation(origin = {6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  flight6DOF1.Force = F;
  flight6DOF1.Moment = M;
end Test6DOF;