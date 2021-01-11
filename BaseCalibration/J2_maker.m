function J = J2_maker(R_BW,P_W,R_BH)
%J2_maker another method to generate Jacobian matrix for the iterative
%algorithm
%   J = J2_maker(R_BW,P_W,R_BH)
%   R_BW:       rotation matrix from base to world, 3x3
%   P_W:        position of marker in world frame, 3xM
%   R_BH:       rotation matrix from base to hand, 3x3xM 

n=size(P_W,2);%number of measurements

crossRP_W=zeros(3,3,n);

for i=1:n
    crossRP_W(:,:,i)=-R_BW*skew(P_W(:,i));
end

J1=[reshape(crossRP_W(:,1,:),[3*n,1]),reshape(crossRP_W(:,2,:),[3*n,1]),reshape(crossRP_W(:,3,:),[3*n,1])];
J2=repmat(eye(3),[n,1]);
J3=-[reshape(R_BH(:,1,:),[3*n,1]),reshape(R_BH(:,2,:),[3*n,1]),reshape(R_BH(:,3,:),[3*n,1])];

J=[J1,J2,J3];
