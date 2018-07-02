function [F] = EOM_bif(U)  
      M		=U(1)          
      alpha	=U(2)
      beta 	=U(3)
      p		=U(4)
      q		=U(5)
      r		=U(6)
      phi  	=U(7)
      theta	=U(8)
              
      gamma    =asin(cos(alpha)*cos(beta)*sin(theta)-sin(beta)*sin(phi)*cos(theta)-sin(alpha)*cos(beta)*cos(phi)*cos(theta))
      mu       =asin((1/cos(gamma))*(sin(theta)*cos(alpha)*sin(beta)+sin(phi)*cos(theta)*cos(beta)-sin(alpha)*sin(beta)*cos(phi)*cos(theta)))
       
	   Factor	=57.29577951
       g		=9.81
	   Vs		=340
	   rho		=1.2256
	   m		=15118.35
	   S		=37.16
	   b		=11.405
	   c		=3.511
	   Ixx		=31181.88
	   Iyy		=205113.07
	   Izz		=230400.22
	   Tm		=49817.6
       
             
             eta= 0.713785
          deltae=-0.16785           
          deltar=0
          deltaa=0
       
       ! CD =01423;
       ! CL=0.732;
       ! CY=Cl=Cn=0;
       ! Cm=-0.1885;
        
          
          
       CD		= 0.0013  *(alpha*Factor)^2   - 0.00438*(alpha*Factor) +0.1423
       CY		=-0.0186  *(beta *Factor)     +(deltaa*Factor/25)*(-0.00227*alpha*Factor+0.039) + (deltar*Factor/30)*(-0.00265*alpha*Factor+0.141)
       CL		=0.0751   *(alpha*Factor)     + 0.0144 *(deltae*Factor)+0.732
       Cl		=(-0.00012*alpha*Factor-0.00092)*beta*Factor-0.0315*p + 0.0126*r+(deltaa*Factor/25)*(0.00121*alpha*Factor-0.0628)*-(deltar*Factor/30)*(0.000351*alpha*Factor-0.0124)
       Cm		=-0.00437 *(alpha*Factor) -0.0196*(deltae*Factor)  -0.123*q -0.1885
       Cn		= 0.00125 *beta*Factor-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474)
       
       F(1)     =(Tm*eta*cos(alpha)*cos(beta)-0.5*CD*rho*(Vs*M)^2*S-m*g*sin(gamma))/(m*Vs)
       F(2)     =q-((p*cos(alpha)+r*sin(alpha))*sin(beta)+(1/(m*Vs*M))*(Tm*eta*sin(alpha)+0.5*CL*rho*(Vs*M)^2*S-m*g*cos(mu)*cos(gamma)))/cos(beta)
       F(3)     =1/(m*Vs*M)*(-Tm*eta*cos(alpha)*sin(beta)+ 0.5*CY*rho*(Vs*M)^2*S+m*g*sin(mu)*cos(gamma))+p*sin(alpha)-r*cos(alpha)
       F(4)     =(Iyy-Izz)/Ixx*q*r+(0.5/Ixx)*rho*(Vs*M)^2*S*b*Cl
       F(5)     =(Izz-Ixx)/Iyy*p*r+(0.5/Iyy)*rho*(Vs*M)^2*S*c*Cm
       F(6)     =(Ixx-Iyy)/Izz*p*q+(0.5/Izz)*rho*(Vs*M)^2*S*b*Cn
       F(7)     =p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta)
       F(8)     =q*cos(phi)-r*sin(phi)
       
end

