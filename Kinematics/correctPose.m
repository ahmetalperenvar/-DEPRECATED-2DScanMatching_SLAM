function [ Rob ] = correctPose( Rob, R, t )
%CORRECTPOSE Apply the 2D rotation R and translation t to the robot and
%generates the uncertainty 

global currentFrame

if currentFrame==1
    return
end

oldor = Rob.state.x_full(3,currentFrame-1);

% reference in the correct axis. ie. the Reference Scan point of view
reff = r2Rt(oldor,[0 0])*[t(1); t(2); 1];

Rob.state.x(1:2)  = Rob.state.x(1:2) + reff(1:2); 
Rob.state.x(3) = normAngle(Rob.state.x(3) + R);
end

