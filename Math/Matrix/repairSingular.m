function [ A ] = repairSingular( A )
%REPAIRSINGULAR Fix a non singular matrix. Uses the idea of

% P. Biber and W. Straβer. The normal distribution transform: a new approach
% to laser scan matching. In Proceedings of the IEEE/RSJ International Con-
% ference on Intelligent Robots and Systems (IROS), volume 3, pages 2743 –
% 2748, October 2003.


if nargin > 1 && repair == 1
%we want to se the smallest eigen values to be 0.001 times the biggest one    
    e(imin) = e(imax) * 0.001;
    A = (e*D)\e;
end

end

