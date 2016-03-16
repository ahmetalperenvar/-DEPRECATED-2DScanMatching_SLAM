function [G] = odo2_cart(F,u)

% ODO2_CART Odometry evolution: The odometry model is in the form
%     u = [x y yaw]

dv = u(3);  
dx = u(1);
dy = u(2); 

G(1:2) = [F(1) + dx; F(2) + dy]; % Position update
G(3) = normAngle(dv+F(3));       %Orientation update

G=G';