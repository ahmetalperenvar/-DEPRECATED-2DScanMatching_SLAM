function [ d P ] = pt2lineDistance( p1, p2, p3 )
%PT2LINEDISTANCE return the distance between 1 point and a line give in
%slope intercept form.

% [m q] = pts2lineSI(p2,p3);
% m1 = -1;
% q1 = p1(2) - m1*p1(2);
% 
% A=[1 -m;1 -m1];
% 
% ip = (A\[q;q1])';
% 
% if isnan(ip(1)) || isinf(ip(1))
%     d = inf;
%     P= [];
%     return
% end
% 
% dff = ip - p2;
% d1 = sqrt(dff(1)^2 +dff(2)^2);
% 
% dff = ip - p3;
% d2 = sqrt(dff(1)^2 +dff(2)^2);
% 
% [mm im] = min([d1,d2]);
% 
% dff = ip - p1;
% d3 = sqrt(dff(1)^2 +dff(2)^2);
% 
% d = d3;
% 
%     
%     if im == 1
%         P = p2;
%         dff = p1 - p2;
%         d = sqrt(dff(1)^2 +dff(2)^2);
%     else
%         P = p3;
%         dff = p1 - p3;
%         d = sqrt(dff(1)^2 +dff(2)^2);
%     end
%    
% if inSegment(ip,p2,p3)
%     d = d3;
% end
%     



xu = p1(1) - p2(1);
yu = p1(2) - p2(2);
xv = p3(1) - p2(1);
yv = p3(2) - p2(2);

d1 = sqrt( (xu)^2 + (yu)^2);
if (xu * xv + yu * yv < 0)
d=d1;
P=p2;
return
end

xu = p1(1) - p3(1);
yu = p1(2) - p3(2);
xv = -xv;
yv = -yv;

d2=sqrt( (xu)^2 + (yu)^2 );
if (xu * xv + yu * yv < 0)
d=d2;
P=p3;
return
end

pts = [p2; p3]; ds=[d1 d2];

d = abs( ( p1(1) * ( p2(2) - p3(2) ) + p1(2) * ( p3(1) - p2(1) ) + ( p2(1) * p3(2) - p2(1) * p2(2) ) ) / sqrt( ( p3(1) - p2(1) )^2 + ( p3(2) - p2(2) )^2 ) );
[mm ii] = min(ds);
P=pts(ii,:);
end
