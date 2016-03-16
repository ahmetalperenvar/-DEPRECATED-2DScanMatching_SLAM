% Function Precompute
% Preproces an Scan for being used in a scan matching algorithm
% In:   
%   Scan: TScan struct scan data in polar and cartesian
% Out:
%   s: struct with precomputed stuff
%       s.refdq
%       s.refdq2
%       s.distRef
%       s.refdqxdqy

function [s] = Precompute(scan)

Refdq = [];
for i = 1:size(scan.polar,2)-1
    Refdq = [Refdq scan.cart(:,i)-scan.cart(:,i+1)];
end

Refdq2 = Refdq.^2;
DistRef = Refdq2(1,:)+Refdq2(2,:);
Refdqxdqy = Refdq(1,:).*Refdq(2,:);

s = struct('refdq',Refdq,'refdq2',Refdq2,'distRef',DistRef,'refdqxdqy',Refdqxdqy);