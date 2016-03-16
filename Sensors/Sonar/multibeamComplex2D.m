function [ localCart2D localPolar2D  ] = multibeamComplex2D( Rob,World,Sen )
%MULTIBEAMCOMPLEX2D Summary of this function goes here
%   Detailed explanation goes here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Multibeam parameters

Fr = Rob.state;
%Multibeam Coorindate System
Q = Fr.gt; %+Rob.state0([1 2 3 6]); %frame ground truth
T = Q(1:2); %robot pose
rad = Q(3); %orientation

nBeams = Sen.par.nBeams;
S = Sen.par; %Sensor parameter

sonarRange = S.maxRange;
sonarWidth = S.maxWidth;
skipped = 0;
localPolar2D=[];
localCart2D=[];
sonarAngle = sonarWidth/nBeams;
noise = Sen.par.beamStd(1);
angle =  -sonarWidth / 2;

for i = 1:nBeams
    
    beamangle = deg2rad(angle + i*sonarAngle);
    angle = normAngle(beamangle+rad);
    
    Tr(1) = T(1) + cos(angle) * sonarRange;
    Tr(2) = T(2) + sin(angle) * sonarRange;
    
    [xout yout] = polyxpoly(World.segments.coord(:,1),World.segments.coord(:,2),...
        [T(1) Tr(1)],[T(2) Tr(2)]);
    
    
    if(isempty(xout) )
        skipped = skipped +1;
        continue;
    end
    
    if size(xout,1) > 1
        mind = inf;
        for k = 1:size(xout,1)
            d = ptsDistance([xout(k) yout(k)],T(1:2));
            if(d < mind)
                cxout = xout(k);
                cyout = yout(k);
                mind = d;
            end
        end
    else
        cxout = xout;
        cyout = yout;
    end
    
    d = norm([cxout-T(1),cyout-T(2)]);
    
    ns = stdErr(0,noise);
    ns2 = stdErr(0,noise);
    outl = randn^2 < Sen.par.outliers;
    
    if outl
        outlp = randn ;
    else
        outlp = 1;
    end
    cxout = cxout+(ns*d)*cos(beamangle);
    cyout = cyout+(ns*d)*sin(beamangle);
    
    localCart = cartProject([cxout cyout ], -rad, (e2R([0 0 -rad])*[-(T(1:2)*outlp)' 1]')' );

    %localCart = cartProject([cxout cyout ], rad(3), (e2R([0 0 rad(3)])*[(T(1:2)*outlp)' 1]')' );
    
    
    % Return the points locally referenced
    localCart2D(i-skipped,1:2) = [localCart(1:2)];
    [la lr] = cart2pol(localCart2D(i-skipped,1), localCart2D(i-skipped,2) );
    localPolar2D(i-skipped,1:2) = [ la lr  ];
    
end


%Draw profile
% figure
%  hold on
%  axis equal
%  displayPoints(localCart2D,'k');
% % displayPoints(globalCart2D,'r');
% % displayPoints(globalPolar2D,'g',1);
%  displayPoints(localPolar2D,'b',1);


end

