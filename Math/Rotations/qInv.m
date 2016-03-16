function [ r ] = qInv( q )
% Compute the invere of a given quaternion q

[a,b,c,d] = split(q);
q2 = (a^2 + b^2 + c^2 + d^2 );
r = [a -b -c -d]./q2;

end

