function [ np ] = polarProject( p, r, t)
%POLARPROJECT Using rotation r and translation t project the Polar point p 
% P has the form [theta range]

        th = normAngle(p(1) + r);

        rc = cos(th)*p(2) + t(1);
        rs = sin(th)*p(2) + t(2);
        
        np(2) = sqrt(rc^2 + rs^2);
        np(1) = atan2(rs,rc);
end

