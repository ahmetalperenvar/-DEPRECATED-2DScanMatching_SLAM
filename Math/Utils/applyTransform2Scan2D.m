function [ A ] = applyTransform2Scan2D( B, R, t )
%APPLYTRANSFORM2SCAN Apply a transformation t and rotation R to the set of
%points defined in B
yaw = R;
x = t(1);
y = t(2);
A = zeros(size(B,1),size(B,2));
sizeB = size(B,1);
    rTn = [cos(R) -sin(R) x; sin(yaw) cos(yaw) y; 0 0 1];

    for i = 1:sizeB
        A(i,1:3) = (rTn*[B(i,1:2) 1]')' ;
    end
end

