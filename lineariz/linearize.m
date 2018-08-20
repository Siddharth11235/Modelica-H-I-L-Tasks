clear
clc
% computation of Jacobian matrix. It returns a matrix which contains
    % partial derivatives of F(x1,x2,Ws) at Xk [Xk]
    
Jacob_mat = Num_Jacobian('fun_trim_wind_level', [39.8858,0.1,0,0,0,100]);
n_st = 6;
    % compute system matrices A and B
    A_mat = Jacob_mat(:,1:n_st);
