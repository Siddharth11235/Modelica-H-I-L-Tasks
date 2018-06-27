clear all
clc
[t,x]=ode45(@EOM_wind,[0 15],[0.3081034 0 0 0 0 0 100 0 100 0 0 0]);

plot(t,x(:,1));
		xlabel('Time, s'), ylabel('Mach'), grid
        title('Aircraft speed')
figure
subplot(2,1,1);
plot(t,x(:,2)*180/pi());
		xlabel('Time, s'), ylabel('alpha (deg)'), grid
        title('Angle of attack,(alpha)')
subplot(2,1,2)
        plot(t,x(:,3)*180/pi());
		xlabel('Time, s'), ylabel('beta (deg)'), grid
        title('side-slip,(beta)')
figure
subplot(3,1,1);
plot(t,x(:,4)*180/pi());
		xlabel('Time, s'), ylabel('Roll Rate (p), deg/s'), grid
        title('Body-Axis Roll Component of Inertial Rate, p')
subplot(3,1,2);
plot(t,x(:,5)*180/pi());
		xlabel('Time, s'), ylabel('Pitch Rate (q), deg/s'), grid
        title('Body-Axis pitch Component of Inertial Rate, q')
subplot(3,1,3);
plot(t,x(:,6)*180/pi());
		xlabel('Time, s'), ylabel('Yaw Rate (r), deg/s'), grid
        title('Body-Axis Yaw Component of Inertial Rate, r')
figure
subplot(3,1,1);
plot(t,x(:,7));
		xlabel('Time, s'), ylabel('North (x), m'), grid
        title('North location, x')
subplot(3,1,2);
plot(t,x(:,8));
		xlabel('Time, s'), ylabel('East (y), m'), grid
        title('East Location, y')
subplot(3,1,3);
plot(t,x(:,9));
		xlabel('Time, s'), ylabel('Altitude (z), m'), grid
        title('Altitude, z')

figure
subplot(3,1,1);
plot(t,x(:,10)*180/pi());
		xlabel('Time, s'), ylabel('Roll Angle (phi), deg'), grid
        title('Earth-Relative Roll Attitude')
subplot(3,1,2);
plot(t,x(:,11)*180/pi());
		xlabel('Time, s'), ylabel('Pitch Angle (theta), deg'), grid
        title('Earth-Relative Ptich Attitude')
subplot(3,1,3);
plot(t,x(:,12)*180/pi());
		xlabel('Time, s'), ylabel('Yaw Angle (psi), deg'), grid
        title('Earth-Relative Yaw Attitude')
figure
%plot3(x(:,7),x(:,8),x(:,9))
plot(x(:,7),x(:,8))

figure
plot(x(:,7),x(:,9))