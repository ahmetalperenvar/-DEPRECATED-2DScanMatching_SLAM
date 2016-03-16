function [ a r ] = polarReference( a,r,t,rot )
%POLARREFERENCE given a point in polar coordinate a, r, apply an axis
%change using the translation t and rotation rot

            rc = cos(rot+a)*r + t(1);
            rs = sin(rot+a)*r + t(2);
            
            r = sqrt(rc^2 + rs^2);
            a = atan2(rs,rc);

end

