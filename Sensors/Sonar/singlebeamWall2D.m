%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Multibeam simulation in 2D
% Simulates a multibeam sonar scanning the wall of the world

function [ localCart2D localPolar2D Sen] = multibeamWall2D(Fr,World,Sen)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Multibeam parameters


%Multibeam Coorindate System
Q = Fr.x; %frame
T = Q(1:3); %robot pose
q = Q(4:7); %quaternion orientation 
rad = q2e(q); %radians orientation

S = Sen.par; %Sensor parameter
S.i = S.i + (1*S.dir);

if(S.i >= S.maxWidth)
    S.dir  = -S.dir; 
end


[localPolar realPoints] = intersectPoint2DWall( [ T(1:2)' rad(3) ], S.i ,World.lims.xMax,World.lims.yMax,S,Sen.par.beamStd(1));

    % Return the points locally referenced
    localPolar2D = localPolar(1:2) ;
    [lx ly] = pol2cart(localPolar2D(1), localPolar2D(2) );
    localCart2D = [ lx ly  T(3) ];
%Draw profile

Sen.par = S;

if localPolar2D(1) == -1 && localPolar2D(2) == -1
    localCart2D = [];
    localPolar2D = [];
end

end
