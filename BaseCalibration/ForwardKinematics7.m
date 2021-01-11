function T = ForwardKinematics7 (Q, xi, gst)
%ForwardKinematics7 Forward kinematics for a seven-axis robot
%
%   T = ForwardKinematics7 (Q, xi, gst) calculates the forward kinematics
%   of a seven-axis robot
%   Q:      7x1 joint variable
%   xi:     6x7 joint twist
%   gst:    4x4 initial homogeneous transformation from base to tool
%   T:      4x4 homogeneous transformation from base to tool

T = se3Exp(xi(:,1)*Q(1))*se3Exp(xi(:,2)*Q(2))*se3Exp(xi(:,3)*Q(3))*se3Exp(xi(:,4)*Q(4))*se3Exp(xi(:,5)*Q(5))*se3Exp(xi(:,6)*Q(6))*se3Exp(xi(:,7)*Q(7))*gst;

end