clear all
clc
syms psi theta phi u v w
mat1=[cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
      sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
     -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi);]

mat2=[u;v;w]
mat=mat1*mat2

pdot=(Iyy-Izz)/Ixx*q*r+(0.5/Ixx*rho*V^2*S*b*Cl)
qdot=(Izz-Ixx)/Iyy*p*r+(0.5/Iyy*rho*V^2*S*c*Cm)
rdot=(Ixx-Iyy)/Izz*p*q+(0.5/Izz*rho*V^2*S*b*Cn)

xdot=mat(1)
ydot-mat(2)
zdot=mat(3)

phidot  = p+q*tan(theta)*sin(phi)+r*tan(theta)*cos(phi)
thetadot= q*cos(phi)-r*sin(phi)
psidot  = q*sec(theta)*sin(phi)+r*sec(theta)*cos(phi)

udot=r*v-q*w+0.5/m*rho*V^2*S*Cx+T/m-g*sin(theta)
vdot=p*w-r*u+0.5/m*rho*V^2*S*Cy+g*sin(phi)*cos(theta)
wdot=q*u-p*v+0.5/m*rho*V^2*S*Cz+g*cos(phi)*cos(theta)

