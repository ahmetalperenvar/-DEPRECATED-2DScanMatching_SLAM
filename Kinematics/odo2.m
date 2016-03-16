function [G, incr] = odo2(F,u)

% ODO2 Odometry evolution: The odometry model is in the form
%     u = [v x yaw]
%     with v and x being the angular and radial motion information in polar
%     coordinates and yaw is the orientation shift. Note that the angular
%     odometry and yaw are indipendent even though they are similar.

dv = u(2);  % Linear and Angular odometry model
dx = u(1);
dy = u(3);  % Yaw shift

incr(1) = dx*cos(dv+F(4));
incr(2) = dx*sin(dv+F(4));
incr(3) = dy;

G(1:3) = [F(1) + incr(1); F(2) + incr(2); F(3)]; % Position update
G(4) = normAngle(incr(3)+F(4)); %Orientation update

G=G';