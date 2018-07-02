%	FLIGHT --  Generic 6-DOF Trim, Linear Model, and Flight Path Simulation

%	June 12, 2015  
%	===============================================================
%	Copyright 2006-2015 by ROBERT F. STENGEL.  All rights reserved.

	clear
	global GEAR CONHIS SPOIL u x V tuHis deluHis uInc TrimHist SMI MODEL RUNNING

    disp('** 6-DOF FLIGHT Simulation **')
    date
    
%	This is the SCRIPT FILE.  It contains the Main Program, which:
%		Defines initial conditions
%		Contains aerodynamic data tables (if required)
%		Calculates longitudinal trim condition
%		Calculates stability-and-control derivatives
%		Simulates flight path using nonlinear equations of motion
		
%	Functions used by FLIGHT:
%		AeroModelAlpha.m   High-Alpha, Low-Mach aerodynamic coefficients of the aircraft, 
%                          thrust model, and geometric and inertial properties
%		AeroModelMach.m    Low-Alpha, High-Mach aerodynamic coefficients of the aircraft, 
%                          thrust model, and geometric and inertial properties
%		Atmos.m            Air density, sound speed
%		DCM.m              Direction-cosine matrix
%		EoM.m              Equations of motion for integration
%		LinModel.m         Equations of motion for linear model definition
%		TrimCost.m         Cost function for trim solution
%		WindField.m        Wind velocity components

%	DEFINITION OF THE STATE VECTOR
%		x(1)    = 		Body-axis x inertial velocity, ub, m/s
%		x(2)    =		Body-axis y inertial velocity, vb, m/s
%		x(3)    =		Body-axis z inertial velocity, wb, m/s
%		x(4)    =		North position of center of mass WRT Earth, xe, m
%		x(5)    =		East position of center of mass WRT Earth, ye, m
%		x(6)    =		Negative of c.m. altitude WRT Earth, ze = -h, m
%		x(7)    =		Body-axis roll rate, pr, rad/s
%		x(8)    =		Body-axis pitch rate, qr, rad/s
%		x(9)    =		Body-axis yaw rate, rr,rad/s
%		x(10)   =		Roll angle of body WRT Earth, phir, rad
%		x(11)   =		Pitch angle of body WRT Earth, thetar, rad
%		x(12)   =		Yaw angle of body WRT Earth, psir, rad
	
%	DEFINITION OF THE CONTROL VECTOR
%		u(1)    = 		Elevator, dEr, rad, positive: trailing edge down
%		u(2)    = 		Aileron, dAr, rad, positive: left trailing edge down
%		u(3)    = 		Rudder, dRr, rad, positive: trailing edge left
%		u(4)    = 		Throttle, dT, %
%		u(5)    =		Asymmetric Spoiler, dASr, rad
%		u(6)    =		Flap, dFr, rad
%		u(7)    =		Stabilator, dSr, rad

    

%   ======================================================================
%	USER INPUTS
%	===========

%	FLIGHT Flags (1 = ON, 0 = OFF)

    MODEL   =   1;      % Aerodynamic model selection
                        % 0: Incompressible flow, high angle of attack
                        % 1: Compressible flow, low angle of attack    
	TRIM    = 	1;		% Trim flag (= 1 to calculate trim @ I.C.)
	LINEAR  = 	0;		% Linear model flag (= 1 to calculate and store F and G)
	SIMUL   =	1;		% Flight path flag (= 1 for nonlinear simulation)
	GEAR    = 	0;		% Landing gear DOWN (= 1) or UP (= 0)
	SPOIL   =	0;		% Symmetric Spoiler DEPLOYED (= 1) or CLOSED (= 0)
	CONHIS  =	1;		% Control history ON (= 1) or OFF (= 0)
	dF      = 	0;		% Flap setting, deg
	RUNNING =   0;      % internal flag, -
    
%	Initial Altitude (ft), Indicated Airspeed (kt), 
%   Dynamic Pressure (N/m^2), and True Airspeed (m/s

	hft         =   10000   % Altitude above Sea Level, ft
    VKIAS       =   150     % Indicated Airspeed, kt
    
        hm          =   hft * 0.3048    % Altitude above Sea Level, m
        VmsIAS      =   VKIAS * 0.5154  % Indicated Airspeed, m/s
    
        [airDens,airPres,temp,soundSpeed] = Atmos(hm)
    
        qBarSL  =   0.5*1.225*VmsIAS^2  % Dynamic Pressure at sea level, N/m^2
        V   =   sqrt(2*qBarSL/airDens);	% True Airspeed, TAS, m/s
        TASms   =   V

%	Alphabetical List of Initial Conditions

	alpha   =	0;      % Angle of attack, deg (relative to air mass)
	beta    =	0;      % Sideslip angle, deg (relative to air mass)
	dA      =	0;      % Aileron angle, deg
	dAS     =	0;      % Asymmetric spoiler angle, deg
	dE      =	0;      % Elevator angle, deg
	dR      =	0;      % Rudder angle, deg
	dS      = 	0;      % Stabilator setting, deg
	dT      = 	0;      % Throttle setting, % / 100
	hdot    =	0;      % Altitude rate, m/s
	p       =	0;      % Body-axis roll rate, deg/s
	phi     =	0;      % Body roll angle wrt earth, deg
	psi     =	0;      % Body yaw angle wrt earth, deg
	q       =	0;      % Body-axis pitch rate, deg/sec
	r       =	0;      % Body-axis yaw rate, deg/s
	SMI     =	0;      % Static margin increment due to center-of-mass variation from reference, %/100
	tf      =	100;    % Final time for simulation, sec
	ti      = 	0;      % Initial time for simulation, sec
	theta   =	alpha;  % Body pitch angle wrt earth, deg [theta = alpha if hdot = 0]
	xe      =	0;      % Initial longitudinal position, m
	ye      = 	0;      % Initial lateral position, m
	ze      = 	-hm;    % Initial vertical position, m [h: + up, z: + down]
    
    if MODEL == 0
        disp('Low-Mach, High-Alpha Model')
    else
        disp('High-Mach, Low Alpha Aerodynamic Model')
    end
    

%	Initial Conditions Depending on Prior Initial Conditions

	gamma	=	57.2957795 * atan(hdot / sqrt(V^2 - hdot^2))
						% Inertial Vertical Flight Path Angle, deg
	qbar	= 	0.5 * airDens * V^2	
						% Dynamic Pressure, N/m^2
	IAS		=	sqrt(2 * qbar / 1.225)
						% Indicated Air Speed, m/s
	Mach	= 	V / soundSpeed	
						% Mach Number
											
%	Initial State Perturbation (Test Inputs: m, m/s, rad, or rad/s)
	delx	=	[0;0;0
				0;0;0
                0;0;0
				0;0;0];
            
%	Initial Control Perturbation (Test Inputs: rad or 100%)			
	delu	=	[0;0;0;-0.234;0;0;0];
	
%	Control Perturbation History (Test Inputs: rad or 100%)
%   =======================================================
%   Each control effector represented by a column
%	Each row contains control increment delta-u(t) at time t:

	tuHis	=	[0 33 67 100];
	deluHis	=	[0 0 0 0 0 0 0
				0 0 0 0 0 0 0
			    0 0 0 0 0 0 0
			    0 0 0 0 0 0 0];
	uInc	=	[];

%	State Vector and Control Initialization, rad
	phir	=	phi * .01745329;
	thetar	=	theta * .01745329;
	psir	=	psi * .01745329;

	windb	=	WindField(-ze,phir,thetar,psir);
	alphar	=	alpha * .01745329;
	betar	=	beta * .01745329;

	x	=	[V * cos(alphar) * cos(betar) - windb(1)
			V * sin(betar) - windb(2)
			V * sin(alphar) * cos(betar) - windb(3)
			xe
			ye
			ze
			p * 0.01745329
			q * 0.01745329
			r * 0.01745329
			phir
			thetar
			psir];
	
	u	=	[dE * 0.01745329
			dA * 0.01745329
			dR * 0.01745329
			dT
			dAS * 0.01745329
			dF * 0.01745329
			dS * 0.01745329];

%	Trim Calculation (for Steady Level Flight at Initial V and h)
%   =============================================================
%   Trim Parameter Vector (OptParam):
%			1 = Stabilator, rad
%			2 = Throttle, %
%			3 = Pitch Angle, rad

	if TRIM >= 1
		'Calculate TRIM Stabilator, Thrust, and Pitch Angle'
        OptParam        =   [];
        TrimHist        =   [];
		InitParam		=	[0.0369;0.1892;0.0986];
        
		[OptParam,J,ExitFlag,Output]  =	fminsearch('TrimCost',InitParam)
%		Optimizing Trim Error Cost with respect to dSr, dT, and Theta
        TrimHist;
        Index=  [1:length(TrimHist)];
        TrimStabDeg     =   57.2957795*OptParam(1)
		TrimThrusPer    =   100*OptParam(2)
        TrimPitchDeg    =   57.2957795*OptParam(3)
        TrimAlphaDeg    =   TrimPitchDeg - gamma
        
%		Insert trim values in nominal control and state vectors
        'Incorporate trim values in control and state vectors'
		u	=	[u(1)
				u(2)
				u(3)
				OptParam(2)
				u(5)
				u(6)
				OptParam(1)]
		format long			
		x	=	[V * cos(OptParam(3))
				x(2)
				V * sin(OptParam(3))
				x(4)
				x(5)
				x(6)
				x(7)
				x(8)
				x(9)
				x(10)
				OptParam(3)
				x(12)]
		format short
	end
		figure
		subplot(1,2,1)
		plot(Index,TrimHist(1,:),Index,TrimHist(2,:),Index,TrimHist(3,:)), legend('Stabilator', 'Thrust', 'Pitch Angle')
		xlabel('Iterations'), ylabel('Stabilator(blue), Thrust(green), Pitch Angle(red)'), grid
        title('Trim Parameters'), legend('Stabilator, rad', 'Thrust, 100%', 'Pitch Angle, rad')
		subplot(1,2,2)
		semilogy(Index,TrimHist(4,:))
		xlabel('Iterations'), ylabel('Trim Cost'), grid
        title('Trim Cost')
        
%	Stability-and-Control Derivative Calculation
%   ============================================
   	if LINEAR >= 1
		disp('Generate and Save LINEAR MODEL')
		thresh	=	[.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1];
		xj		=	[x;u];
		xdotj		=	LinModel(ti,xj);
		[dFdX,fac]	=	numjac('LinModel',ti,xj,xdotj,thresh,[],0);
		Fmodel		=	dFdX(1:12,1:12)
		Gmodel		=	dFdX(1:12,13:19)
		save ('Fmodel','Fmodel','TASms','hm')
		save ('Gmodel','Gmodel')
    end

%	Flight Path Calculation
%   =======================
    if SIMUL >= 1
        RUNNING =   1;  
		tspan	=	[ti tf];
		xo		=	x + delx
		u		=	u + delu
        
        options =   odeset('Events',@event,'RelTol',1e-7,'AbsTol',1e-7);
		[t,x]	=	ode15s(@EoM,tspan,xo,options);
        
		kHis	=	length(t)

%       Plot Control History
        figure
        subplot(2,2,1)
        plot(tuHis, 57.29578*deluHis(:,1) + 57.29578*delu(1), tuHis, 57.29578*deluHis(:,7) + 57.29578*delu(7))    
        xlabel('Time, s'), ylabel('Elevator (blue), Stabilator (green), deg'), grid
        title('Pitch Test Inputs'), legend('Elevator, dE', 'Stabilator, dS')
        subplot(2,2,2)
        plot(tuHis, 57.29578*deluHis(:,2) + 57.29578*delu(2), tuHis, 57.29578*deluHis(:,3) + 57.29578*delu(3), tuHis, 57.29578*deluHis(:,5) + 57.29578*delu(5))    
        xlabel('Time, s'), ylabel('Aileron (blue), Rudder (green), Asymmetric Spoiler (red), deg'), grid
        title('Lateral-Directional Test Inputs'), legend('Aileron, dA', 'Rudder, dR', 'Asymmetric Spoiler, dAS')
        subplot(2,2,3)
        plot(tuHis, deluHis(:,4) + delu(4))    
        xlabel('Time, s'), ylabel('Throttle Setting'), grid
        title('Throttle Test Inputs')
        subplot(2,2,4)
        plot(tuHis, 57.29578*deluHis(6) + 57.29578*delu(6))    
        xlabel('Time, s'), ylabel('Flap, deg'), grid
        title('Flap Test Inputs')        
%       Plot State History
		figure
		subplot(2,2,1)
		plot(t,x(:,1))
		xlabel('Time, s'), ylabel('Axial Velocity (u), m/s'), grid
        title('Forward Body-Axis Component of Inertial Velocity, u')
		subplot(2,2,2)
		plot(t,x(:,2))
		xlabel('Time, s'), ylabel('Side Velocity (v), m/s'), grid
        title('Side Body-Axis Component of Inertial Velocity, v')
		subplot(2,2,3)
		plot(t,x(:,3))
		xlabel('Time, s'), ylabel('Normal Velocity (w), m/s'), grid
        title('Normal Body-Axis Component of Inertial Velocity, z')
		subplot(2,2,4)
		plot(t,x(:,1),t,x(:,2),t,x(:,3))
		xlabel('Time, s'), ylabel('u (blue), v (green), w (red), m/s'), grid
        title('Body-Axis Component of Inertial Velocity') 
        legend('Axial velocity, u', 'Side velocity, v', 'Normal velocity, w') 
        
		figure
        subplot(3,2,1)
		plot(t,x(:,4))
		xlabel('Time, s'), ylabel('North (x), m'), grid
        title('North Location, x')
        
		subplot(3,2,2)
		plot(t,x(:,5))
		xlabel('Time, s'), ylabel('East (y), m'), grid
        title('East Location, y')
        
		subplot(3,2,3)
		plot(t,-x(:,6))
		xlabel('Time, s'), ylabel('Altitude (-z), m'), grid
        title('Altitude, -z')
        
		subplot(3,2,4)
		plot((sqrt(x(:,4).*x(:,4) + x(:,5).*x(:,5))),-x(:,6))
		xlabel('Ground Range, m'), ylabel('Altitude, m'), grid
        title('Altitude vs. Ground Range')
        
        subplot(3,2,5)
        plot(x(:,4),x(:,5))
		xlabel('North, m'), ylabel('East, m'), grid
        title('Ground Track, North vs. East')
        
        subplot(3,2,6)
        plot3(x(:,4),x(:,5),-x(:,6))
		xlabel('North, m'), ylabel('East, m'), zlabel('Altitude, m'), grid
        title('3D Flight Path')
        
        figure
		subplot(2,2,1)
		plot(t,x(:,7) * 57.29578)
		xlabel('Time, s'), ylabel('Roll Rate (p), deg/s'), grid
        title('Body-Axis Roll Component of Inertial Rate, p')
		subplot(2,2,2)
		plot(t,x(:,8) * 57.29578)
		xlabel('Time, s'), ylabel('Pitch Rate (q), deg/s'), grid
        title('Body-Axis Pitch Component of Inertial Rate, q')
		subplot(2,2,3)
		plot(t,x(:,9) * 57.29578)
		xlabel('Time, s'), ylabel('Yaw Rate (r), deg/s'), grid
        title('Body-Axis Yaw Component of Inertial Rate, r')
        subplot(2,2,4)
		plot(t,x(:,7) * 57.29578,t,x(:,8) * 57.29578,t,x(:,9) * 57.29578)
		xlabel('Time, s'), ylabel('p (blue), q (green), r (red), deg/s'), grid
        title('Body-Axis Inertial Rate Vector Components')
        legend('Roll rate, p', 'Pitch rate, q', 'Yaw rate, r')
        
        figure
		subplot(2,2,1)
		plot(t,x(:,10) * 57.29578)
		xlabel('Time, s'), ylabel('Roll Angle (phi), deg'), grid
        title('Earth-Relative Roll Attitude')
		subplot(2,2,2)
		plot(t,x(:,11) * 57.29578)
		xlabel('Time, s'), ylabel('Pitch Angle (theta), deg'), grid
        title('Earth-Relative Pitch Attitude')
		subplot(2,2,3)
		plot(t,x(:,12) * 57.29578)
		xlabel('Time, s'), ylabel('Yaw Angle (psi, deg'), grid
        title('Earth-Relative Yaw Attitude')
		subplot(2,2,4)
		plot(t,x(:,10) * 57.29578,t,x(:,11) * 57.29578,t,x(:,12) * 57.29578)
		xlabel('Time, s'), ylabel('phi (blue), theta (green), psi (red), deg'), grid
        title('Euler Angles')
        legend('Roll angle, phi', 'Pitch angle, theta', 'Yaw angle, psi')
        
        VAirRel         =   [];
        vEarth          =   [];
        AlphaAR         =   [];
        BetaAR          =   [];
        windBody        =   [];
        airDensHis      =   [];
        soundSpeedHis   =   [];
        qbarHis         =   [];
        GammaHis        =   [];
        XiHis           =   [];
        
        for i = 1:kHis
            windb           =	WindField(-x(i,6),x(i,10),x(i,11),x(i,12));
            windBody        =   [windBody windb];
            [airDens,airPres,temp,soundSpeed] = Atmos(-x(i,6));
            airDensHis      =   [airDensHis airDens];
            soundSpeedHis   =   [soundSpeedHis soundSpeed];
        end
        
        vBody           =   [x(:,1) x(:,2) x(:,3)]';
        vBodyAir        =   vBody + windBody;

        for i = 1:kHis
            vE          =   DCM(x(i,10),x(i,11),x(i,12))' * [vBody(1,i);vBody(2,i);vBody(3,i)];
            VER         =   sqrt(vE(1)^2 + vE(2)^2 + vE(3)^2);
            VAR         =   sqrt(vBodyAir(1,i)^2 + vBodyAir(2,i)^2 + vBodyAir(3,i)^2);
            VARB        =   sqrt(vBodyAir(1,i)^2 + vBodyAir(3,i)^2);

            if vBodyAir(1,i) >= 0
                Alphar      =	asin(vBodyAir(3,i) / VARB);
            else
                Alphar      =	pi - asin(vBodyAir(3,i) / VARB);
            end
            
            AlphaAR     =   [AlphaAR Alphar];
            Betar       = 	asin(vBodyAir(2,i) / VAR);
            BetaAR      =   [BetaAR Betar];
            vEarth      =   [vEarth vE];
            Gammar      =   asin(-vEarth(3,i) / VER);
            GammaHis    =   [GammaHis Gammar];
            Xir         =   asin(vEarth(2,i) / sqrt((vEarth(1,i))^2 + (vEarth(2,i))^2));
            if vEarth(1,i) <= 0 && vEarth(2,i) <= 0
                Xir = -pi - Xir;
            end
            if vEarth(1,i) <= 0 && vEarth(2,i) >= 0
                Xir = pi - Xir;
            end
            
            XiHis       =   [XiHis Xir];
            VAirRel     =   [VAirRel VAR];
        end

        MachHis         =   VAirRel ./ soundSpeedHis;
        AlphaDegHis 	=	57.2957795 * AlphaAR;
        BetaDegHis      =	57.2957795 * BetaAR;
        qbarHis         =	0.5 * airDensHis .* VAirRel.*VAirRel;
        GammaDegHis     =   57.2957795 * GammaHis;
        XiDegHis        =   57.2957795 * XiHis;
        
        figure
        subplot(3,1,1)
        plot(t, VAirRel')    
        xlabel('Time, s'), ylabel('Air-relative Speed, m/s'), grid
        title('True AirSpeed, Vair')
        subplot(3,1,2)
        plot(t, MachHis')    
        xlabel('Time, s'), ylabel('M'), grid
        title('Mach Number, M')
        subplot(3,1,3)
        plot(t, qbarHis')    
        xlabel('Time, s'), ylabel('qbar, N/m^2'), grid
        title('Dynamic Pressure, qbar')
        
        figure
        subplot(2,1,1)
        plot(t, AlphaDegHis', t, BetaDegHis')    
        xlabel('Time, s'), ylabel('Angle of Attack, deg (blue), Sideslip Angle, deg (green)'), grid
        title('Aerodynamic Angles'), legend('Angle of Attack, alpha', 'Sideslip Angle, beta')
        subplot(2,1,2)
        plot(t, GammaDegHis', t, XiDegHis')    
        xlabel('Time, s'), ylabel('Vertical, deg (blue), Horizontal, deg (green)'), grid
        title('Flight Path Angles'), legend('Flight Path Angle, gamma', 'Heading Angle, psi')
        
    'End of FLIGHT Simulation'
	end
