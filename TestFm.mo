model TestFm
  Real al = 1;
  Real be = 1;
  Real del[3] = {0.0,0,0};
  Real Thrust[3] = {0 , 1, -9.8};
  ForceMoment_Gen forceMoment_Gen1 annotation(
    Placement(visible = true, transformation(origin = {-2, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  forceMoment_Gen1.alpha  = al;
  forceMoment_Gen1.beta = be;
  forceMoment_Gen1.Thrust = Thrust;
  forceMoment_Gen1.delta = del;
  
end TestFm;