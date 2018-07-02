function [XD] = EOM_body(t,X)
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
Factor=57.29577951;
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
             
             eta=0.713785;        
          deltae=-0.16785;           
          deltar=0;
          deltaa=0;

          alpha=atan(w/u);
           beta=asin(v/V);
%      CD1: -5 <= alpha <= 20;  !CD2: 20 <= alpha <= 40;
	   CD1=0.0013*(alpha*Factor)^2-0.00438*(alpha*Factor)+0.1423;
	   CD2=-0.0000348*(alpha*Factor)^2+0.0473*(alpha*Factor)-0.3580;
	   CY=-0.0186*(beta*Factor)+(deltaa*Factor/25)*(-0.00227*alpha*Factor+0.039)+(deltar*Factor/30)*(-0.00265*alpha*Factor+0.141);
	   
% 	   CL1: -5 <= alpha <= 10;    CL2: 10 <= alpha <= 40
	   CL1=0.0751*(alpha*Factor)+0.0144*(deltae*Factor)+0.732;	 
	   CL2=-0.00148*(alpha*Factor)^2+0.106*(alpha*Factor)+0.0144*(deltae*Factor)+0.569;				  
	   
% 	   Cl1: -5 <=alpha <= 15 deg; Cl2: 15 <= alpha <=25
	   Cl1=(-0.00012*alpha*Factor-0.00092)*beta*Factor-0.0315*p+0.0126*r+(deltaa*Factor/25)*(0.00121*alpha*Factor-0.0628)-(deltar*Factor/30)*(0.000351*alpha*Factor-0.0124);
       Cl2=(0.00022*alpha*Factor-0.006)*beta*Factor-0.0315*p+0.0126*r+(deltaa*Factor/25)*(0.00121*alpha*Factor-0.0628)-(deltar*Factor/30)*(0.000351*alpha*Factor-0.0124);
	   
	   Cm=-0.00437*(alpha*Factor)-0.0196*(deltae*Factor)-0.123*q-0.1885;
	   
% 	   Cn1: -5 <=alpha <= 10 deg; Cn2: 10 <= alpha <=25;   Cn3: 25 <= alpha <=35
	   Cn1=0.00125 *beta*Factor-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474);
	   Cn2=(-0.00022*alpha*Factor+0.00342)*(beta*Factor)-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474);
	   Cn3=-0.00201*beta*Factor-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474);
       
      if(alpha < 0.1745329252)
       CD=CD1;
       CY=CY;
       CL=CL1;
       Cl=Cl1;
       Cm=Cm;
       Cn=Cn1;
      elseif(alpha >= 0.1745329252 && alpha < 0.2617993878)
       CD=CD1;
       CY=CY;
       CL=CL2;
       Cl=Cl1;
       Cm=Cm;
       Cn=Cn2;    
      elseif(alpha >= 0.2617993878 && alpha < 0.3490658504)
       CD=CD1;
       CY=CY;
       CL=CL2;
       Cl=Cl2;
       Cm=Cm;
       Cn=Cn2;    
      elseif(alpha >= 0.3490658504 && alpha < 0.436332313)
       CD=CD2;
       CY=CY;
       CL=CL2;
       Cl=Cl2;
       Cm=Cm;
       Cn=Cn2;
      else
       CD=CD2;
       CY=CY;
       CL=CL2;
       Cl=Cl2;
       Cm=Cm;
       Cn=Cn3;
      end  

 CX=sin(alpha)*CL-cos(alpha)*CD;
 CY=CY;
 CZ=-cos(alpha)*CL-sin(alpha)*CD; 

 gamma=asin(cos(alpha)*cos(beta)*sin(theta)-sin(beta)*sin(phi)*cos(theta)-sin(alpha)*cos(beta)*cos(phi)*cos(theta));
  chi=acos((cos(psi)*cos(theta)*cos(alpha)*cos(beta)+sin(beta)*(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))+sin(alpha)*cos(beta)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)))/cos(gamma));
   mu=asin((1/cos(gamma))*(sin(theta)*cos(alpha)*sin(beta)+sin(phi)*cos(theta)*cos(beta)-sin(alpha)*sin(beta)*cos(phi)*cos(theta)));     
assignin('base', 'mu', mute);

 % u v w p q r x y z phi theta psi
XD =[r*v-q*w+0.5/m*rho*V^2*S*CX+T/m-g*sin(theta);
     p*w-r*u+0.5/m*rho*V^2*S*CY+g*sin(phi)*cos(theta);
     q*u-p*v+0.5/m*rho*V^2*S*CZ+g*cos(phi)*cos(theta); 
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

