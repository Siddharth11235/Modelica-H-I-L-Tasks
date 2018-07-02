function dydt = ver_loop_simp( t,y)
dydt=zeros(3,1);
V=41.13;
dydt(1)=1;
%dydt(1)=(y(3)*cos(y(1))+y(2)*sin(y(1)));
dydt(2)=V*cos(y(1));
dydt(3)=-V*sin(y(1));
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


end

