function [out] = normAngle(angle)
%Normalize an angle in the range [-pi/2;pi/2]
out = angle+2*pi*floor((pi-angle)/(2*pi));