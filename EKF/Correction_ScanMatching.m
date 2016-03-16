function [ x, P ] = Correction_ScanMatching( v, motion, x, P )
%CORRECTION_SCANMATCHING Summary of this function goes here
%   Detailed explanation goes here

v(3) = normAngle(v(3));

S = motion.Pq + diag(diag(ones(3)))*0.01;

K = (P*motion.H')/(S);
x = x + K * v;

%Joseph Form
P=(eye(length(x))-K*motion.H)*P;
x(3) = normAngle(x(3));

end

