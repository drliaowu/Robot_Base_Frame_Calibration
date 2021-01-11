function g = LBRfkine(theta)
%LBRfkine Forward kinematics of KUKA LBR iiwa robot
%
%   g = LBRfkine (theta) calculates the forward kinematics of KUKA LBR iiwa
%   robot
%   theta:  7x1 joint variable
%   g:      4x4 homogeneous transformation

% w: axis direction
% p: point on axis
w1=[0;0;1]; p1=[0;0;0];
w2=[0;1;0]; p2=[0;0;0.360];
w3=[0;0;1]; p3=[0;0;0];
w4=[0;-1;0]; p4=[0;0;0.780];
w5=[1;0;0]; p5=[0;0;0.780];
w6=[0;1;0]; p6=[0.400;0;0.780];
w7=[0;0;-1]; p7=[0.400;0;0.780];

% xi: joint twists
xi1=[w1;cross(p1,w1)];
xi2=[w2;cross(p2,w2)];
xi3=[w3;cross(p3,w3)];
xi4=[w4;cross(p4,w4)];
xi5=[w5;cross(p5,w5)];
xi6=[w6;cross(p6,w6)];
xi7=[w7;cross(p7,w7)];
xi=[xi1,xi2,xi3,xi4,xi5,xi6,xi7];

% gst: initial transformation from base to tool
gst=[-1 0 0 0.400;
    0 1 0 0;
    0 0 -1 0.669;
    0 0 0 1];

% forward kinematics
g=ForwardKinematics7(theta,xi,gst);
