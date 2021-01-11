function [ R_BW, t_BW, P_H ] = closedForm( R_BH, t_BH, P_W )
%closedForm Closed form solution for finding the kinematic base frame by
%hand-eye calibration using 3D position data
%
%   [ R_BW, t_BW, P_H ] = closedForm( R_BH, t_BH, P_W )
%   R_BH:   rotation matrix from base to hand, 3x3xM 
%   t_BH:   translation vector from base to hand, 3xM
%   P_W:    position of marker in world frame, 3xM
%   R_BW:   rotation matrix from base to world, 3x3
%   t_BW:   translantion vector from base to world, 3x1
%   P_H:    position of marker in hand frame, 3x1

M = size(R_BH,3); %number of measurements
A = zeros(3*M,15);
b = zeros(3*M,1);
for i=1:M
    A(i*3-2:i*3,:)=[kron(transpose(P_W(:,i)),eye(3)),eye(3),-R_BH(:,:,i)];
    b(i*3-2:i*3,1)=t_BH(:,i);
end
x = A\b;
tempR_BW=reshape(x(1:9),3,3);
t_BW=x(10:12);
P_H=x(13:15);
[U,~,V]=svd(tempR_BW);
R_BW=U*V';
end

