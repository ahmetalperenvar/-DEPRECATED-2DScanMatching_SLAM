function [ f ] = frameRef( f1, f2, invs )
%FRAMEREF Reference the frame f1 with respect to f2. f1 and f2 are vector
% containing a position x y z and a rotation Yaw

if (nargin == 3 && invs == 0) || nargin == 2
    f = r2Rt(f2(3),f2(1:2))*[f1(1:2); 1];
    f = [f(1:2); normAngle(f1(3)+f2(3))];
else
    rt = r2R(-f2(3))*[f2(1:2); 1];
    f = r2Rt(-f2(3),-rt(1:2))*[f1(1:2); 1];
    f = [f(1:2); normAngle(f1(3)-f2(3))];
end

% Transform back to 4D state

end

