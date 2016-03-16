function [ s in ] = pts2lineSI( p1, p2 )
%PTS2LINESI Returns the slope intercept of the line passing between the two
%points P1 and P2

d = p1 - p2;
s = d(1) / d(2);
in = p1(2) - s*p1(1);

end

