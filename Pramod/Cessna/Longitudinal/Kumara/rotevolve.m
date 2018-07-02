function c=rotevolve(c0,w,T)
%function for evolving the rotation matrix with a constant
%body-referenced angular velocity
%c0=rotation matrix at t=0
%w=angular velocity vector (3x1) (rad/s)
%T=final time (s)
%(c)2006 Ashish Tewari
S=[0 -w(3,1) w(2,1);w(3,1) 0 -w(1,1);-w(2,1) w(1,1) 0];
dt=2*pi/(10^6*norm(w))
cdt=eye(3)-S*dt;
t=dt;
c=cdt*c0;
while t<=T
c=cdt*c;
t=t+dt;
end

