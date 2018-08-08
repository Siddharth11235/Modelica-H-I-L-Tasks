model Linearise_Test

import Modelica_LinearSystems2.Utilities.Import.*;

Real A[:,:] "A-matrix";
Real B[:,:] "B-matrix";
Real C[:,:] "C-matrix";
Real D[:,:] "D-matrix";
String inputNames[:] "Modelica names of inputs";
String outputNames[:] "Modelica names of outputs";
String stateNames[:] "Modelica names of states";

algorithm
(A,B,C,D,inputNames,outputNames,stateNames) :=linearize(
    "Wind6DOFVer");


end Linearise_Test;