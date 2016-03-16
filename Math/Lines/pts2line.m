function [ m d ] = pts2line( p2,p3 )
%PTS2LINE Summary of this function goes here
%  Find the line passing between two points. Return in slope point
%  form.
dx = p3(1) - p2(1);
dy = p3(2) - p2(2);
m = dy / dx ;
%d = sqrt(dx^2 + dy^2);
d = -m*p3(1) + p3(2);

end
