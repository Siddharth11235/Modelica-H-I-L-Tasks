
model val_gen

import Modelica.SIunits.*;
import Modelica.Math.Matrices.*;
// lift

parameter Real CD0 = 0.01631, CD_alpha = 0.2108, CD_beta = 0, CD_delta_e = 0.3045, CD_q = 0, CL0 = 0.09167, CL_alpha = 3.5016, CL_delta_e = 0.2724, CL_q = 2.8932, Cl_beta = -0.02854, Cl_delta_a = 0.1682, Cl_delta_r = 0, Cl_p = -0.3209, Cl_r = 0.03066, Cm0 = -0.02338, Cm_alpha = -0.5675, Cm_delta_e = -0.3254, Cm_q = -1.3990, Cn_beta = -0.00040, Cn_delta_a = -0.00328, Cn_delta_r = 0, Cn_p = -0.01297, Cn_r = -0.00434, Cy_beta = -0.07359, Cy_delta_a = 0,Cy_delta_r = 0, Cy_p = 0, Cy_r = 0,  b= 1.4224, cbar = 0.3302, g = 9.81, m = 1.56, rho = 1.225, s = 0.2589;


parameter Real[3] W =m * {0, 0, 9.81};
parameter Real[3,3] J = {{0.1147, 0, -0.0015}, {0, 0.0576, 0}, {-0.0015, 0, 0.1712}};
Real L;
Real D;

Real Q;
  

 Real V;

parameter Real[3] omega = {0,0.0,0};

Real CL;
Real CD;
parameter Real alpha = 0.06;
Real de;
Real thrust;
Real theta = alpha;


Real x=sin(30*time); 
Real dummy(start=0.0, fixed=true), nextDummy(fixed=true, start=1e-6); 

equation


when time>=pre(nextDummy) then 
    dummy = pre(nextDummy); 
    nextDummy = 2*pre(nextDummy); // Force solver to reset at t=1,2,4,8 .... * 1e-3 
end when; 



Q=0.5*rho*V^2;

0  = Cm0+Cm_alpha*alpha+((Cm_q*omega[2]*cbar)/(2*V))+Cm_delta_e*de;
CL = CL0+CL_alpha*alpha+((CL_q*omega[2]*cbar)/(2*V))+CL_delta_e*de;
CD = CD0+CD_alpha*alpha+((CD_q*omega[2]*cbar)/(2*V))+CD_delta_e*abs(de);// + CDbeta * beta + CDdeltae * Elevator;
// forces and moments

L = CL*s*Q;
D = CD*s*Q;



0 = -D*cos(alpha)+L*sin(alpha)+thrust - m*g*sin(theta);
0 = -D*sin(alpha)-L*cos(alpha)+m*g*cos(theta);


//(0.5*rho*V^2)*s*CL -m*g = 0;
//thrust*cos(alpha) - (0.5*rho*V^2)*s*CD = 0;
end val_gen;