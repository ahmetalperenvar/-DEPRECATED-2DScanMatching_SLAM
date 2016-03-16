function [u] = calcOdo2D(pos1,pos2)
%Calculate the 2D odometry shift from two 4D poses keeping fixed the z-axis
%and using the yaw as orientation. The information returned is a polar
%odometry information and an orientation change.
%     u = [v x yaw]

% Displacement
dif = pos1(1:2) - pos2(1:2);
difa = (atan2(dif(2),dif(1)) );
dify = normAngle(pos1(3) - pos2(3));
dif = sqrt( (dif(1))^2 + (dif(2))^2);

u = [ dif difa dify];

end
