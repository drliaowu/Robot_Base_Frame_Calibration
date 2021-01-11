function J = J_maker(R_BW,P_W,R_BH) 
%J_maker Jacobian matrix for the iterative algorithm
%
%   J = J_maker(R_BW,P_W,R_BH)
%   R_BW:       rotation matrix from base to world, 3x3
%   P_W:        position of marker in world frame, 3xM
%   R_BH:       rotation matrix from base to hand, 3x3xM 

n=size(P_W,2);%number of measurements

w = vlogR(R_BW);
W = skew(w);
theta = norm(w);

crossRP_W=zeros(3,3,n);
Q_BW = R_BW'*(eye(3)+1/theta^2*(1-cos(theta))*W+1/theta^3*(theta-sin(theta))*W*W);

for i=1:n
    crossRP_W(:,:,i)=-R_BW*skew(P_W(:,i))*Q_BW;
end

J1=[reshape(crossRP_W(:,1,:),[3*n,1]),reshape(crossRP_W(:,2,:),[3*n,1]),reshape(crossRP_W(:,3,:),[3*n,1])];
J2=repmat(eye(3),[n,1]);
J3=-[reshape(R_BH(:,1,:),[3*n,1]),reshape(R_BH(:,2,:),[3*n,1]),reshape(R_BH(:,3,:),[3*n,1])];

J=[J1,J2,J3];