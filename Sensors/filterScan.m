function [ Rob ] = filterScan( Rob, Sen, Tim, Opt )
%FILTERSCAN Filters and form a Scan depending on the paremeters

% Scan Building: compound the scan taken at different locations.
% The scan stored in Rob.raw represents the
% scan taken from the time frame Robot.filtered.last till now. If the
% number of scan taken per pose is bigger than one, it is implicit that
% those scan are taken at the same position. Only works for the 2D case.
% The points are shifted over the 2 axis and rotated using the yaw odometry

lastfilter = Rob.filtered.last+1; %last filtering
npose = floor((Tim.currentFrame-lastfilter) / Tim.step) + 1;  %number of pose involved
poses = Rob.state.x;
if( npose > 1)
    poses = [Rob.Map.prev.x(:,lastfilter:lastfilter+npose-2) poses];    %poses involved
end
nscans = size(Sen.raw.localCart,1);     % number of scans taken
scanpose = floor(nscans/npose);                    % number of scans per pose
centertraj = poses(:,floor(npose/2)+1);
x = centertraj(1:2)';
yaw = centertraj(3);

Rob.filtered.localCart = [];
Rob.filtered.localPolar = [];


% Reference every scan point to the same coordinate system, which is the
% center of the poses involved


for i = 1:npose
    
    curpose = poses(:,i);
    
    xn = curpose(1:2)';
    yawn = curpose(3);
    u = calcOdo2D( [xn yawn], [x yaw] );
    rr = r2Rt(-u(3),-[u(1) u(2)]);

    intv = 1+(i-1)*scanpose: 1+(i)*scanpose-1;
    % Less than one point per pose
    if scanpose <= 1 %&& i~=npose

            localpoints = Sen.raw.timed(lastfilter+i-1).localCart;
            polarlocalpoints = Sen.raw.timed(lastfilter+i-1).localPolar;
            
            for zz = 1:size(localpoints,1)
                scanpol = polarProject(polarlocalpoints(zz,:), u(3), u(1:2) );
                scan =  rr*localpoints(zz,:)'  ;
                
                Rob.filtered.localCart = [Rob.filtered.localCart; scan'];
                Rob.filtered.localPolar = [Rob.filtered.localPolar; scanpol];
            end
       
        
    else
        % 1 ore more points per pose
        for k = intv

            Rob.filtered.localCart = [ Rob.filtered.localCart; (Sen.raw.localCart(k,:) ) ];
            Rob.filtered.localPolar = [Rob.filtered.localPolar; Sen.raw.localPolar(k,:) ]; %polarProject(Sen.raw.localPolar(k,:), u(3), u(1:2) ) ]; 

        end
    end
    
    
    
end

%Calculate the points uncertainty

std = [Sen.par.beamStd(1) 0; 0 Sen.par.beamStd(2)];
Rob.filtered.covm = [];

%calculate the covariance matrix of the measurements
for j = 1:size(Rob.filtered.localCart,1)
    J = [cos(Rob.filtered.localPolar(j,1)) ...
        -Rob.filtered.localPolar(j,2)*sin(Rob.filtered.localPolar(j,1));...
        sin(Rob.filtered.localPolar(j,1))...
        Rob.filtered.localPolar(j,2)*cos(Rob.filtered.localPolar(j,1))];
    Rob.filtered.covm(:,:,j) = J*std*J';
end

Rob.filtered.last = Tim.currentFrame;
%Rob.state.x = centertraj;
Rob.filtered.pos =  centertraj;
Rob.filtered.poses = [Rob.filtered.poses; centertraj];
Rob.filtered.timePoses = [Rob.filtered.timePoses Tim.currentFrame];

% Filter SCAN : TO DO
