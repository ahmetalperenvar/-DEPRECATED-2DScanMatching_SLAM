function [ d ] = ptsDistance( p1, p2 )
%2PTSDISTANCE distance between 2 points

dx = p2(1) - p1(1);
dy = p2(2) - p1(2);
d = sqrt(dx^2 + dy^2);
end
