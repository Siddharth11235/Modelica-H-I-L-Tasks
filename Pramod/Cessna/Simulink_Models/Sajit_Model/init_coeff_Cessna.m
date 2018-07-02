%% environmental constants
rho=1.225; % air desnsity 
g = 9.8; % gravity

%% geometric quantities
c_bar=1.493; % chord length
b=10.911; % wing span
S=16.1651; % wing area

%% mass and inertia
mass = 1043.26; % mass
Ixx=1285.31;
Ixy=0.0;
Iyx=Ixy;
Ixz=0.0;
Izx=Ixz;
Iyy= 1824.93; % moment of inertia
Iyz=0.0;
Izy=Iyz;
Izz=2666.893;

%weight
W = mass*g;

%% aerodynamic coefficients
% drag
cd0=0.036;
cd_alpha = 0.13;
k_drag=0.0830304;
cd_delta_e=0.06;
cd_beta=0.17;

%side force
cy_beta=-0.31;
cy_p=-0.037;
cy_r=0.21;
cy_delta_r=0.187;
cy_delta_a=0;

% lift
cl0=0.25;   
cl_alpha=4.47;
cl_q=3.9;
cl_delta_e=0.3476;

% rolling moment
c_l_beta=-0.089;
c_l_p=-0.47;
c_l_r=0.096;
c_l_delta_a=-0.09;
c_l_delta_r=0.0147;

% pitching moment
cm0=-0.02;
cm_alpha=-1.8;
cm_alpha_dot=0.0;
cm_q=-12.4;
cm_delta_e=-1.28;
alpha_dot=0;

% yawing moment
cn_beta=0.065;
cn_p=-0.03;
cn_r=-0.99;
cn_delta_a=-0.0053;
cn_delta_r=-0.0657;
