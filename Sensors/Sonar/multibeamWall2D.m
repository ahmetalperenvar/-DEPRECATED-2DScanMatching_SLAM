%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Multibeam simulation in 2D
% Simulates a multibeam sonar scanning the wall of the world

function [localCart2D localPolar2D ] = multibeamWall2D(Fr,World,Sen)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Multibeam parameters

Fr = Fr.frame;
%Multibeam Coorindate System
Q = Fr.x; %frame ground truth
T = Q(1:3); %robot pose
q = Q(4:7); %quaternion orientation 
rad = q2e(q); %radians orientation
nBeams = Sen.par.nBeams;
S = Sen.par; %Sensor parameter


skipped = 0;
localPolar2D=[];
localCart2D=[];
for i = 1:nBeams 
    
    %Calculate the intersection with the walls defined in the world point using the ground truth
    [localPolar realPoints] = intersectPoint2DWall( [ T(1:2)' rad(3) ], i ,World.lims.xMax,World.lims.yMax,S,Sen.par.beamStd(1));
    
   
    
    if(localPolar(1) == -1 && localPolar(2) == -1)
        skipped = skipped +1;
        continue;
    end
    % Return the points locally referenced
    localPolar2D(i-skipped,1:2) = localPolar(1:2) ;
    [lx ly] = pol2cart(localPolar2D(i-skipped,1), localPolar2D(i-skipped,2) );
    localCart2D(i-skipped,1:3) = [ lx ly  T(3) ];

end


%Draw profile
% figure
% hold on
% axis equal
% displayPoints(localCart2D,'k');
% displayPoints(globalCart2D,'r');
% displayPoints(globalPolar2D,'g',1);
%displayPoints(localPolar2D,'b',1);

end
