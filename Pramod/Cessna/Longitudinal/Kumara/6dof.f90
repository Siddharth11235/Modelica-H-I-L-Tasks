!----------------------------------------------------------------------
!----------------------------------------------------------------------
!   ab :            The A --> B reaction 
!----------------------------------------------------------------------
!----------------------------------------------------------------------

      SUBROUTINE FUNC(NDIM,U,ICP,PAR,IJAC,F,DFDU,DFDP)
!     ---------- ----

! Evaluates the algebraic equations or ODE right hand side

! Input arguments :
!      NDIM   :   Dimension of the ODE system 
!      U      :   State variables
!      ICP    :   Array indicating the free parameter(s)
!      PAR    :   Equation parameters

! Values to be returned :
!      F      :   ODE right hand side values

! Normally unused Jacobian arguments : IJAC, DFDU, DFDP (see manual)

      IMPLICIT NONE
      INTEGER NDIM, IJAC, ICP(*)
      DOUBLE PRECISION U(NDIM), PAR(*), F(NDIM), DFDU(*), DFDP(*)
      DOUBLE PRECISION M,alpha,beta,p,q,r,phi,theta
      DOUBLE PRECISION eta,deltae,deltar,deltaa
	  DOUBLE PRECISION g,Vs,rho,m,S,b,c,Ixx,Iyy,Izz,Tm,CD,CY,CL,Cl,Cm,Cn,gamma,mu,Factor
	  
       M		=U(1)
       alpha	=U(2)
       beta 	=U(3)
       p		=U(4)
       q		=U(5)
       r		=U(6)
       phi  	=U(7)
       theta	=U(8)

       eta		=PAR(1)
	   deltae	=PAR(2)
       deltar	=PAR(3)
       delata	=PAR(4)
       
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
	   
	   !CD1: -5 <= alpha <= 20;  !CD2: 20 <= alpha <= 40;
	   CD1		= 0.0013   *(alpha*Factor)**2  - 0.00438*(alpha*Factor)+0.1423
	   CD2		=-0.0000348*(alpha*Factor)**2  + 0.0473 *(alpha*Factor)-0.3580
	   
	   CY		=-0.0186  *(beta *Factor)  +(deltaa*Factor/25)*(-0.00227*alpha*Factor+0.039) + (deltar*Factor/30)*(-0.00265*alpha*Factor+0.141)
	   
	   !CL1: -5 <= alpha <= 10;    CL2: 10 <= alpha <= 40
	   CL1		= 0.0751   *(alpha*Factor)     + 0.0144 *(deltae*Factor)+0.732	 
	   CL2		=-0.00148  *(alpha*Factor)**2  + 0.106	*(alpha*Factor) +0.0144*(deltae*Factor)+0.569				  
	   
	   !Cl1: -5 <=alpha <= 15 deg; Cl2: 15 <= alpha <=25
	   Cl1		=(-0.00012*alpha*Factor-0.00092)*beta*Factor-0.0315*p + 0.0126*r+(deltaa*Factor/25)*(0.00121*alpha*Factor-0.0628)*-(deltar*Factor/30)*(0.000351*alpha*Factor-0.0124)					  
	   Cl2		=( 0.00022 *alpha*Factor-0.006) *beta*Factor-0.0315*p + 0.0126*r+(deltaa*Factor/25)*(0.00121*alpha*Factor-0.0628)*-(deltar*Factor/30)*(0.000351*alpha*Factor-0.0124)
	   
	   Cm		=-0.00437 *(alpha*Factor) -0.0196*(deltae*Factor)  -0.123*q -0.1885
	   
	   !Cn1: -5 <=alpha <= 10 deg; Cn2: 10 <= alpha <=25;   Cn3: 25 <= alpha <=35
	   Cn1		=  0.00125 *beta*Factor-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474)
	   Cn2		=(-0.00022*alpha*Factor+0.00342)*(beta*Factor)-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474)									   !10 <=alpha <= 25 deg
	   Cn3		= -0.00201*beta*Factor-0.0142*r+(deltaa*Factor/25)*(0.000213*alpha*Factor+0.00128)+(deltar*Factor/30)*(0.000804*alpha*Factor-0.0474)														   !25 <=alpha <= 35 deg
	   

      
       F(1)	=  (Tm*eta*cos(alpha)*cos(beta)-0.5*CD*rho*(Vs*M)**2*S-m*g*sin(gamma))/(m*Vs)
       F(2)	=q-((p*cos(alpha)+r*sin(alpha))*sin(beta)+(1/(m*Vs*M))*(Tm*eta*sin(alpha)+0.5*CL*rho*(Vs*M)**2*S-m*g*cos(mu)*cos(gamma)))/cos(beta)
       F(3)	=1/(m*Vs*M)*(-Tm*eta*cos(alpha)*sin(beta)+ 0.5*CY*rho*(Vs*M)**2*S+m*g*sin(mu)*cos(gamma))+p*sin(alpha)-r*cos(alpha)
       F(4)	=(Iyy-Izz)/Ixx*q*r+(0.5/Ixx)*rho*(Vs*M)**2*S*b*Cl
       F(5)	=(Izz-Ixx)/Iyy*p*r+(0.5/Iyy)*rho*(Vs*M)**2*S*c*Cm
       F(6)	=(Ixx-Iyy)/Izz*p*q+(0.5/Izz)*rho*(Vs*M)**2*S*b*Cn
       F(7)	=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta)
       F(8)	=q*cos(phi)-r*sin(phi)
      END SUBROUTINE FUNC
!----------------------------------------------------------------------
!----------------------------------------------------------------------

      SUBROUTINE STPNT(NDIM,U,PAR,T)
!     ---------- -----

! Input arguments :
!      NDIM   :   Dimension of the ODE system 

! Values to be returned :
!      U      :   A starting solution vector
!      PAR    :   The corresponding equation-parameter values
!      T      :	  Not used here

      IMPLICIT NONE
      INTEGER NDIM
      DOUBLE PRECISION U(NDIM), PAR(*), T

! Initialize the equation parameters
       PAR(1)= 0.713785
       PAR(2)=-0.16785
       PAR(3)= 0
       PAR(4)= 0
! Initialize the solution
       U(1)=0.3081
       U(2)=0
       U(3)=0
       U(4)=0
       U(5)=0
       U(6)=0
       U(7)=0
       U(8)=0

      END SUBROUTINE STPNT
!----------------------------------------------------------------------
!----------------------------------------------------------------------
! The following subroutines are not used here,
! but they must be supplied as dummy routines

      SUBROUTINE BCND 
      END SUBROUTINE BCND

      SUBROUTINE ICND 
      END SUBROUTINE ICND

      SUBROUTINE FOPT 
      END SUBROUTINE FOPT

      SUBROUTINE PVLS
      END SUBROUTINE PVLS
!----------------------------------------------------------------------
!----------------------------------------------------------------------
