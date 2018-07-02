clear all
clc
[t,x]=ode45(@EOM_body,[0 5],[100 0 0 0 0 0 0 0 100 0 0 0]);

% n=size(x);
% for i=1:n(1)
% phi=x(i,10);
% theta=x(i,11);
% psi=x(i,12);
% 
% HIB=zeros(3);
% HIB(1,1)=cos(theta)*cos(psi);
% HIB(2,1)=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
% HIB(3,1)=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
% 
% HIB(1,2)=cos(theta)*sin(psi);
% HIB(2,2)=cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
% HIB(3,2)=-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
% 
% HIB(1,3)=-sin(theta);
% HIB(2,3)=sin(phi)*cos(theta);
% HIB(3,3)=cos(phi)*cos(theta);
% 
% HBI=HIB';
% vb=[x(i,1); x(i,2); x(i,3);];
% ve=HBI*vb;
% 
% vear(i,1)=ve(1,1);
% vear(i,2)=ve(2,1);
% vear(i,3)=ve(3,1);
% 
% end
% 
% 
% figure
% subplot(3,1,1);
% plot(t,x(:,1));
% 	xlabel('Time, s'), ylabel('Axial Velocity (u), m/s'), grid
%     title('Forward Body-Axis Component of Inertial Velocity, u')
% subplot(3,1,2);
% plot(t,x(:,2));
%     xlabel('Time, s'), ylabel('Side Velocity (v), m/s'), grid
%     title('Side Body-Axis Component of Inertial Velocity, v')
% subplot(3,1,3);
% plot(t,x(:,3));
% 	xlabel('Time, s'), ylabel('Normal Velocity (w), m/s'), grid
%     title('Normal Body-Axis Component of Inertial Velocity, w')
% 
% figure
% subplot(3,1,1);
% plot(t,x(:,4)*180/pi());
% 	xlabel('Time, s'), ylabel('Roll Rate (p), deg/s'), grid
%     title('Body-Axis Roll Component of Inertial Rate, p')
% subplot(3,1,2);
% plot(t,x(:,5)*180/pi());
% 	xlabel('Time, s'), ylabel('Pitch Rate (q), deg/s'), grid
%     title('Body-Axis pitch Component of Inertial Rate, q')
% subplot(3,1,3);
% plot(t,x(:,6)*180/pi());
% 	xlabel('Time, s'), ylabel('Yaw Rate (r), deg/s'), grid
%     title('Body-Axis Yaw Component of Inertial Rate, r')
% 
% figure
% subplot(3,1,1);
% plot(t,x(:,7));
% 		xlabel('Time, s'), ylabel('North (x), m'), grid
%         title('North location, x')
% subplot(3,1,2);
% plot(t,x(:,8));
% 		xlabel('Time, s'), ylabel('East (y), m'), grid
%         title('East Location, y')
% subplot(3,1,3);
% plot(t,x(:,9));
% 		xlabel('Time, s'), ylabel('Altitude (z), m'), grid
%         title('Altitude, z')
% 
% figure
% subplot(3,1,1);
% plot(t,x(:,10)*180/pi());
% 		xlabel('Time, s'), ylabel('Roll Angle (phi), deg'), grid
%         title('Earth-Relative Roll Attitude')
% subplot(3,1,2);
% plot(t,x(:,11)*180/pi());
% 		xlabel('Time, s'), ylabel('Pitch Angle (theta), deg'), grid
%         title('Earth-Relative Ptich Attitude')
% subplot(3,1,3);
% plot(t,x(:,12)*180/pi());
% 		xlabel('Time, s'), ylabel('Yaw Angle (psi), deg'), grid
%         title('Earth-Relative Yaw Attitude')