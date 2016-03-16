function [ out ] = inSegment( p1, p2, p3 )
%INSEGMENT returns 1 if the point P1 is part the segment P2, P3

pp1 = sort( [p2(1),p3(1)]);
pp2 = sort( [p2(2),p3(2)]);

out = p1(1) >= pp1(1) && p1(1) <= pp1(2) &&...
        p1(2) >= pp2(1) && p1(2) <= pp2(2);

end

