function [xdot] = mav_de(t,x)

global CD0 CD_Alpha CD_Alpha_Sq CD_Delta_e CD_Delta_e_Sq ...
       CY_Beta CY_p CY_r CY_Delta_a CY_Delta_r...
       CL0 CL_Alpha CL_Alpha_Sq CL_Alpha_Cube CL_Alpha_Four CL_Delta_e CL_Delta_e_Sq ...
       C_l_Beta C_l_p C_l_r C_l_Delta_a C_l_Delta_r  ...
       Cm0 Cm_Alpha Cm_Alpha_Sq Cm_Delta_e Cm_Delta_e_Sq ...
       Cn_Beta Cn_p Cn_r Cn_Delta_a Cn_Delta_r ...
       CT_J0 CT_J CT_J_Sq D_propeller ...
       c_bar b S mass Ixx Ixy Iyx Ixz Izx Iyy Iyz Izy Izz g...
       deltaA deltaE deltaR RPM
   



% States
u = x(1); v = x(2); w = x(3);
p = x(4); q = x(5); r = x(6);
xe = x(7); ye = x(8); ze = x(9);
phi = x(10); theta = x(11); psi = x(12);

alpha = atan(w/u);
V = sqrt(u^2+v^2+w^2);
beta = asin(v/V);




%% Air Density Calculation w.r.t altitude

%%%%%%% Calculation of density with respect to altitude
R = 287;
h = -ze;  % z direction is downwards

if (h <= 11000)  % temperature gradient region
    Temp = 288.16 - 6.5*10^(-3)*h;
    rho = 1.225 * (Temp/288.16)^(-((g/(-6.5*10^(-3)*R))+1));
    
    
elseif (h > 11000 && h <= 25000) % isothermal region
    rho = 0.3639 * exp(-(g/(R*216.66))*(h-11000));
    

elseif (h > 25000 && h <= 45000) % temperature gradient region
    Temp = 216.66 + 3*10^(-3)*(h-25000);
    rho = 0.03946 * (Temp/216.66)^(-1*((g/(0.003*R))+1));
    
else
    rho = 0.0880;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Propulsion Model
%% Propulsion Model
J = V/(RPM*D_propeller);
Thrust_x = (4/pi^2) * (rho * RPM^2 * D_propeller^4) * (CT_J_Sq * J^2 + CT_J * J + CT_J0); % Calculate thrust from propeller rpm

%% Weight and dynamic pressure
W = mass*g; %weight
qd = 0.5*rho*V^2; % Dynamic pressure

%% Normalized velocities in three body axes
p_cap = p*b/(2*V);
q_cap = q*c_bar/(2*V);
r_cap = r*b/(2*V);



%% Coeff. of forces (drag, lift and side force)
C_D = CD0 + CD_Alpha * abs(alpha) + CD_Alpha_Sq * alpha^2 + CD_Delta_e * abs(deltaE) + CD_Delta_e_Sq * deltaE^2;
C_L = CL0 + CL_Alpha * alpha + CL_Alpha_Sq * alpha^2 + CL_Alpha_Cube * alpha^3 + CL_Alpha_Four * alpha^4 + CL_Delta_e * deltaE + CL_Delta_e_Sq * deltaE^2;
C_Y = CY_Beta * beta + CY_p * p_cap + CY_r * r_cap + CY_Delta_a * deltaA + CY_Delta_r * deltaR; 

%% Coeff. of Moments 
C_l = C_l_Beta * beta + C_l_p * p_cap + C_l_r * r_cap + C_l_Delta_a * deltaA + C_l_Delta_r * deltaR; % coeff. of moment about X-axis(tendency to roll)
C_m = Cm0 + Cm_Alpha * alpha + Cm_Alpha_Sq * alpha^2 + Cm_Delta_e * deltaE + Cm_Delta_e_Sq * deltaE^2; % coeff. of moment about Y-axis(tendency to pitch)
C_n = Cn_Beta * beta + Cn_p * p_cap + Cn_r * r_cap + Cn_Delta_a * deltaA + Cn_Delta_r * deltaR; % coeff. of moment about Z-axis(tendency to yaw)


%% Forces - Lift, Drag, Side Force and Weight
L = qd*S*C_L;
D = qd*S*C_D;
Y = qd*S*C_Y;
W = g*mass;

%% Moments
l = qd*S*C_l*b;
m = qd*S*C_m*c_bar;
n = qd*S*C_n*b;


%% Transormation matrix from wind to body axes
trans_wind_body = [cos(alpha)*cos(beta)   -cos(alpha)*sin(beta)   -sin(alpha); ...
    sin(beta)                cos(beta)            0     ; ...
    sin(alpha)*cos(beta)   -sin(alpha)*sin(beta)   cos(alpha)];


%% Forces and Moments in body axes
forces = trans_wind_body*[-D Y -L]' + W*[-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]' + Thrust_x*[1 0  0]';

moments = [l m n]';



%% Dynamic Equations  - From Steven Lewis

% Translation
u_dot = forces(1)/mass + r*v - q*w ;
v_dot = forces(2)/mass + p*w - r*u ;
w_dot = forces(3)/mass + q*u - p*v ;

uvw_dot = [u_dot v_dot w_dot]';

%Rotation
p_dot = (Izz*l + Ixz*n -(Ixz*(Iyy-Ixx-Izz)*p - (Ixz*Ixz+Izz*(Izz-Iyy))*r)*q) / (Ixx*Izz-Ixz*Ixz);
q_dot = m/Iyy - ((Ixx-Izz)/Iyy)*p*q -(Ixz/Iyy)*(p^2 - r^2) ;
r_dot = (Ixz*l + Ixx*n + (Ixz*(Iyy-Ixx-Izz)*r + (Ixz^2+Ixx*(Ixx-Iyy))*p)*q) / (Ixx*Izz-Ixz^2);

% gam = (Ixx*Izz)-Ixz^2;
% 
% k1 = Ixz*(Ixx-Iyy+Izz)/gam;
% k2 = -(Izz*(Izz-Iyy)+Ixz^2)/gam;
% k3 = Izz/gam;
% k4 = Ixz/gam;
% 
% k5 = (Izz-Ixx)/Iyy;
% k6 = -Ixz/Iyy;
% k7 = 1/Iyy;
% 
% k8 = ((Ixx-Iyy)*Ixx+Ixz^2)/gam;
% k9 = -Ixz*(Ixx-Iyy+Izz)/gam;
% k10 = Ixz/gam;
% k11 = Ixx/gam;
% 
% p_dot = k1*p*q + k2*q*r + k3*l + k4*n;
% q_dot = k5*p*q + k6*(p^2-r^2) + k7*m;
% r_dot = k8*p*q + k9*q*r + k10*l + k11*n;
% 
 pqr_dot = [p_dot q_dot r_dot]';

%% Kinematic Equations

% Translation
xe_dot = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
ye_dot = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)); 
ze_dot = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);

xyz_dot = [xe_dot ye_dot ze_dot]';

%Rotation
phi_dot = p+q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi) ;
psi_dot = (q*sin(phi)+r*cos(phi))/cos(theta);

euler_dot = [phi_dot theta_dot psi_dot]';


xdot = [uvw_dot;pqr_dot;xyz_dot;euler_dot];
