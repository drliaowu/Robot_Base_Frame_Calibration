function [ R_BW, t_BW, P_H ] = iterative( R_BH, t_BH, P_W, R_BW_init, t_BW_init, P_H_init )
%iterative Iterative solution for finding the kinematic base frame by
%hand-eye calibration using 3D position data
%
%   [ R_BW, t_BW, P_H ] = closedForm( R_BH, t_BH, P_W )
%   R_BH:       rotation matrix from base to hand, 3x3xM 
%   t_BH:       translation vector from base to hand, 3xM
%   P_W:        position of marker in world frame, 3xM
%   R_BW_init:  initial rotation matrix from base to world, 3x3
%   t_BW_init:  initial translantion vector from base to world, 3x1
%   P_H_init:   initial position of marker in hand frame, 3x1
%   R_BW:       rotation matrix from base to world, 3x3
%   t_BW:       translantion vector from base to world, 3x1
%   P_H:        position of marker in hand frame, 3x1

x=inf;
R_BW = R_BW_init;
t_BW = t_BW_init;
P_H = P_H_init;

while (norm(x)>1e-6)
    
% there are two ways to generate the Jacobian matrix
    J=J_maker(R_BW,P_W,R_BH);
%   J=J2_maker(R_BW,P_W,R_BH);    
    f=f_maker(R_BW, t_BW, P_W, R_BH, t_BH, P_H);

    x=J\f;

%if J2_maker used, comment the first three lines and uncomment the second
%three lines
    w = vlogR(R_BW);
    w = w + x(1:3);
    R_BW = rotationMatrix(w/norm(w),norm(w));
%   delta_theta=x(1:3);
%   delta_R=rotationMatrix(delta_theta/norm(delta_theta),norm(delta_theta));
%   R_BW = R_BW*delta_R; % update rotation 

    delta_t_BW=x(4:6);
    t_BW=t_BW+delta_t_BW;

    delta_P_H=x(7:9);
    P_H=P_H+delta_P_H;

end
end