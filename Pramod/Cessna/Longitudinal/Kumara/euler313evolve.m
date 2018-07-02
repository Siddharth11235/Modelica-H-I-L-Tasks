function xdot=euler313evolve(t,x)
%x(1)=psi, x(2)=theta, x(3)=phi
% (c) 2006 Ashish Tewari
w=[0.1,-0.5,-1];% angular velocity in rad/s
xdot(1,1)=(sin(x(3))*w(1)+cos(x(3))*w(2))/sin(x(2));
xdot(2,1)=cos(x(3))*w(1)-sin(x(3))*w(2);
xdot(3,1)=w(3)-(sin(x(3))*cos(x(2))*w(1)+cos(x(3))*cos(x(2))*w(2))/sin(x(2));

end

