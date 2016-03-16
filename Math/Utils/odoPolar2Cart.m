function [ u2 ] = odoPolar2Cart( u )
%ODOPOLAR2CART transform an odometry in polar format to cartesian format

u2(1) = cos(u(1))*u(2);
u2(2) = sin(u(1))*u(2);

end

