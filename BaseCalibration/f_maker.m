function f = f_maker (R_BW, t_BW, P_W, R_BH, t_BH, P_H)
%f_maker generate f_0 for the iterative algorithm
%
%   f = f_maker (R_BW, t_BW, P_W, R_BH, t_BH, P_H)
%   R_BW:       rotation matrix from base to world, 3x3
%   t_BW:       translantion vector from base to world, 3x1
%   P_W:        position of marker in world frame, 3xM
%   R_BH:       rotation matrix from base to hand, 3x3xM 
%   t_BH:       translation vector from base to hand, 3xM
%   P_H:        position of marker in hand frame, 3x1

n=size(P_W,2);%number of measurements
f=zeros(3,n);
for i=1:n
    f(:,i)=-R_BW*P_W(:,i)-t_BW+R_BH(:,:,i)*P_H+t_BH(:,i);
end
f=reshape(f,[3*n,1]);