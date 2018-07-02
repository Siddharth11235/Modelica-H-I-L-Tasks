function [XD] = EOM(t,X)
     u=X(1);
     v=X(2);
     w=X(3);
     p=X(4);
     q=X(5);
     r=X(6);
     x=X(7);
     y=X(8);
     z=X(9);
   phi=X(10);
 theta=X(11);
   psi=X(12);

V=sqrt(u^2+v^2+w^2);
        g=9.81;
	   Vs=340;
	  rho=1.2256;
	    m=15118.35;
	    S=37.16;
	    b=11.405;
	    c=3.511;
	  Ixx=31181.88;
	  Iyy=205113.07;
	  Izz=230400.22;
	    T=49817.6;
     ! Tm=49817.6;
Cx=0.2; Cy=0.3; Cz=0.2;  Cl=0.1; Cm=0.2;  Cn=0.3;
       
XD =[r*v-q*w+0.5/m*rho*V^2*S*Cx+T/m-g*sin(theta);
     p*w-r*u+0.5/m*rho*V^2*S*Cy+g*sin(phi)*cos(theta);
     q*u-p*v+0.5/m*rho*V^2*S*Cz+g*cos(phi)*cos(theta); 
    (Iyy-Izz)/Ixx*q*r+(0.5/Ixx*rho*V^2*S*b*Cl);
    (Izz-Ixx)/Iyy*p*r+(0.5/Iyy*rho*V^2*S*c*Cm);
    (Ixx-Iyy)/Izz*p*q+(0.5/Izz*rho*V^2*S*b*Cn); 
     w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta);
     v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + u*cos(theta)*sin(psi);
     w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi);
     p+q*tan(theta)*sin(phi)+r*tan(theta)*cos(phi);
     q*cos(phi)-r*sin(phi);
     q*sec(theta)*sin(phi)+r*sec(theta)*cos(phi);];
end

