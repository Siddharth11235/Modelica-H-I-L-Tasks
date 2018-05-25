block AnglesOfAttackandSideSlip

import Modelica.Math.Matrices.*;
import Modelica.SIunits.*;
import Modelica.Blocks.Interfaces.*;
import Modelica.Math.Vectors.*;

RealInput[3] vel annotation(
    Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Velocity
    
//Angle of sideslip

RealOutput alpha annotation(Placement(visible = true, transformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //alpha


RealOutput beta (start = 0)annotation(Placement(visible = true, transformation(origin = {110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //Beta


RealOutput Cw_b[3,3] annotation(Placement(visible = true, transformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0))); //cW_B


Real vw[3] = {0,0,0};
Real vrel[3] = vel - vw;





 equation
 Cw_b  = {{cos(alpha)*cos(beta), sin(beta), sin(alpha)*cos(beta)},{-cos(alpha)*sin(beta), cos(beta), -sin(alpha)*sin(beta)},{-sin(alpha), 0, cos(alpha)}};
 
 alpha= atan2(vrel[3],vel[1]);
 beta  = asin(vrel[2]/norm(vrel));
end AnglesOfAttackandSideSlip;