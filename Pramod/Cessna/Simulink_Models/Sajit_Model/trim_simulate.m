% Sajith Kumar K K, Feb 15, 2015

% Trim and Simulate CESSNA aircraft
% The model has been implmeted using : 1) only blocks, 2) half embedded code, half blocks 3) full embedded code
% use according to convenience, comment the other ones 



clear all;clc;
%% load coefficients and paramters
init_coeff_Cessna

%% Initial Flight Values

% body velocity
u_init=80.00;
v_init=0.00;
w_init=0;

% euler angles
phi_init=0;
theta_init=0.0105;
psi_init=0;

% body angular rates
p_init=0;
q_init=0;
r_init=0;

% position with respect to flat earth (in meteres)
xe_init=0;    
ye_init=0;
ze_init=-200; % downward is positive




 %% trim calculation
  
 % get sequence of states
% model made using  no embedded code, all functions implemented using blocks
%[statenames, stateblocks]=getstatenames('full_block_trim') 
% coefficients and forces,moments calculation done using embedded code, 6DoF equations implemented using block
%[statenames, stateblocks]=getstatenames('half_code_trim')
% all functions implemented using embedded code 
 [statenames, stateblocks]=getstatenames('full_code_trim')


%%%%%%%%%%%%%%  >>>>>>>  Steady Level Flight

 % input --> [deltaE deltaA deltaR T]
 u0 = [0 0 0 1500]';  % initial guess for inputs
 iu = [2 3]';  % fix deltaA = delatR = 0
 
 % output [u v w phi theta psi z  alpha beta gamma V]
 y0 = [80 0 0 0 0 0 -200 0.03 0 0 80]';
 iy = [2 4 6 7 9 10 11]';  % fix v=0(no side velocity) , phi=psi=0 (roll and yaw is zero since level flight), ...
                                  % z=-100 (desired height, z is positive downwards), V=60 (desired velocity)
 
 %states u,v,w,phi,theta,psi,x_e,y_e,z_e,p,q,r
 x0=[80 0 0 0 0 0 0 0 -2 00 0 0 0  ]';
 ix =  [2 4 6 8 9 10 11 12]'; % fix v=0,phi=psi=0,y=0,z=-100, p=q=r=0 (no rotation in any axis)
 %idx tells whichever states for which derviatives need to be zero
 dx0 = [0 0 0 0 0 0 0 0 0 0 0 0]'; 
 idx = [1 2 3 4 5 6 8 9 10 11 12]'; % except xdot, all derivatives are
                                    % fixed to zero
                                       
       
                       
                          
                          
% %%%%%%%%%%%%%%%   >>>>>>> Steady Climb                    
%                        
% u0=[0.1,0,0,1750]';
% iu=[2,3]';
% %outputs: u1,v2,w3,phi4,theta5,psi6,z7,alpha8,beta9,gamma10,airspeed11
% y0=[40,0,0,0,0,0,0,0,0,0.0834,40]';
% iy=[2,4,6,9,10,11]';
% 
% %states u,v,w,phi,theta,psi,x_e,y_e,z_e,p,q,r
% x0=[60,0,0,0,0,0,0,0,0,0,0,0]';
% ix=[2,4,6,8,10,11,12]';
% dx0=[0,0,0,0,0,0,0,0,0,0,0,0]';
% idx=[1,2,3,4,5,6,8,10,11,12]'; % keep constant zdot
%                                    
 
 
%% trim at above menionted values
% model made usingno embedded code, all functions implemented using blocks
%[x_trim,u_trim,y_trim,dx_trim] = trim('full_block_trim',x0,u0,y0,ix,iu,iy,dx0,idx);
% coefficients and forces,moments calculation done using embedded code, 6DoF equations implemented using block
%[x_trim,u_trim,y_trim,dx_trim] = trim('half_code_trim',x0,u0,y0,ix,iu,iy,dx0,idx);
% all functions implemented using embedded code 
 [x_trim,u_trim,y_trim,dx_trim] = trim('full_code_trim',x0,u0,y0,ix,iu,iy,dx0,idx);


 trim_inputs = u_trim 
 trim_velocities = x_trim(1:3) 
 trim_position = x_trim(7:9) 
 trim_body_rates = x_trim(10:12) 
 phi_trim = x_trim(4) 
 theta_trim = x_trim(5) 
 psi_trim = x_trim(6) 
 alpha_trim = y_trim(8) 
 gamma_trim = y_trim(10) 
 trim_velocity = y_trim(11) 
 Thrust_trim = u_trim(4) %Newtons
 deltaE_trim = rad2deg(u_trim(1)) % deg
 

%% Comments on trim values
% for level flight at V = 60 m/s the following values are obtained for inputs:
% if CD = Cd0 + k*CL^2 is used, Thrust_trim = 1516.03N, deltaE_trim = % -1.796deg, deltaA = deltaR =0;  (this is used as default)
% if CD = Cd0 + Cd_alpha*abs(alpha) + Cd_deltaE*abs(deltaE) is used, Thrust_trim = 1388.8N, deltaE_trim = -1.797deg, deltaA = deltaR =0;
% when above equation used with out absolute values of alpha and deltaE, Thrust_trim = 1255.9N, deltaE_trim = -1.798deg, deltaA = deltaR =0;





%% Inputs  
deltaE = u_trim(1);
deltaA = u_trim(2);
deltaR = u_trim(3);
T = u_trim(4);



%% simulate
% model made using no embedded code, all functions implemented using blocks
% open('full_block_model')
% sim('full_block_model')

% coefficients and forces,moments calculation done using embedded code, 6DoF equations implemented using block
% open('half_code_model')
% sim('half_code_model')
% 
% all functions implemented using embedded code 
open('full_code_model')
sim('full_code_model')
