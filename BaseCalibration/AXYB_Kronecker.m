function [ gX, gY] = AXYB_Kronecker( gA, gB )
% solve AX=YB using Kronecker product

%Reference:
%Simultaneous robot-world and hand-eye calibration
%...using dual-quaternions and Kronecker product
%Aiguo Li*, Lin Wang and Defeng Wu

%% solve RX RY
M = size(gA,3);
RA = gA(1:3,1:3,:);
RB = gB(1:3,1:3,:);
tA = reshape(gA(1:3,4,:),[3,M]);
tB = reshape(gB(1:3,4,:),[3,M]);

LeftMat = zeros(12*M,24);
RightVec = zeros(12*M,1);

for i=1:M
    LeftMat(12*i-11:12*i,:) = [kron(RA(:,:,i),eye(3)),-kron(eye(3),transpose(RB(:,:,i))),zeros(9,6);
                               zeros(3,9),           kron(eye(3),transpose(tB(:,i))),  -RA(:,:,i), eye(3)];
    RightVec(12*i-11:12*i,1) = [zeros(9,1);tA(:,i)];
end

element = LeftMat\RightVec;

RX_ = transpose(reshape( element(1:9),[3,3]));
RY_ = transpose(reshape( element(10:18),[3,3]));

RX = RX_;
RY = RY_;

tX = element(19:21);
tY = element(22:24);

gX = [RX, tX;
    0,0,0,1];
gY = [RY, tY;
    0,0,0,1];
end

