

model Flight_Dynamics_Test

import Modelica.Math.Matrices.*;

Real Force[3] = {1,0,0};
Real Moment[3] = {0,0,0};
parameter Real mass = 1;
parameter Real g[3] = {0, 0, -9.8};
parameter Real J[3,3] = {{1, 0, 0},{0,1,0},{0,0,1}};
Real acc[3];
Real vel[3];
Real pos[3];
Real ang_acc[3];
Real ang_vel[3];
Real angles[3];
Real Omega[3,3] = skew(ang_vel);
equation
acc = (1/mass)*Force + g + Omega*vel;
der(vel) = acc;
der(pos) = vel;
ang_acc = inv(J)*(Moment - Omega*vel);
der(ang_vel) = ang_acc;
der(angles) = ang_vel;
end Flight_Dynamics_Test;