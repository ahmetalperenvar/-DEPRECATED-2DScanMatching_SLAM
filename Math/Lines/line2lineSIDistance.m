function [ d P] = line2lineSIDistance(p1, m1, q1, m, q )
%PTS2LINESIDISTANCE Returns the distance from P1 to the line defined by M
%and Q
% m1 = -m;
% q1 = p1(2) - m1*p1(2);

A=[1 -m;1 -m1];

P = (A\[q;q1])';

if isnan(P(1)) || isinf(P(1))
    d = inf;
    P= [];
    return
end

dff = P - p1;
d = sqrt(dff(1)^2 +dff(2)^2);
%if inSegment(ip,p2,p3)

end

