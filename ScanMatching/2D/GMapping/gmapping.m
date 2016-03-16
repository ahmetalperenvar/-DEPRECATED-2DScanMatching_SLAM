% GMAPPING: Gmapping/VascoThis method was
% presented as the VASCO scan matcher in the CARMEN
% framework. Later, it was used by GMapping with minor
% changes. The idea is to search for a better pose in the
% borders of the axis of an uncertainty ellipse. The axis of
% the ellipse have ﬁxed length δd . At every iteration the
% axis length is divided by 2. In [20] authors redeﬁned the
% ellipse size using the pose uncertainty thus reducing the
% computational time by skipping unlikely poses.

% [1] Giorgio Grisetti, Cyrill Stachniss, and Wolfram Burgard: Improved
% Techniques for Grid Mapping with Rao-Blackwellized Particle Filters,
% IEEE Transactions on Robotics, 2006 (link)

% [2] Giorgio Grisetti, Cyrill Stachniss, and Wolfram Burgard:
% Improving Grid-based SLAM with Rao-Blackwellized Particle Filters
% by Adaptive Proposals and Selective Resampling,
% In Proc. of the IEEE International Conference on Robotics and Automation (ICRA), 2005 (link)

% The idea is to apply use a gradient descent method to search for possible
% good poses. The space of search is limited by a boundary

function [ R t NI ] = gmapping( ScanRef, ScanNew, motion )
% In:
%           ScanRef: localCart[2xN], points cartesian
%           ScanRef: localCart[2xN], points cartesian
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations

global Opt DEBUG


niter = Opt.scanmatcher.iterations;

ScanNew = ScanNew.localCart;
ScanRef = ScanRef.localCart;

sizenew = size(ScanNew,1);

% Initial motion estiamtion q0
u = motion.con.u;


x0 = u(1);
y0 = u(2);
yaw0 = u(3);

%Apply the initial estimation to S NEW
ScanNewTR = applyTransform2Scan2D(ScanNew, yaw0, [x0 y0]);

% Make the occupancy grid as big as to contain both scans
[PARAM.xmin PARAM.xmax PARAM.ymin PARAM.ymax] = minmaxScan(ScanRef);
[PARAMN.xmin PARAMN.xmax PARAMN.ymin PARAMN.ymax] = minmaxScan(ScanNewTR);
PARAM.L = Opt.map.resolution;  %Occupancy resolution
PARAM.type = 'binary';
PARAM.xmin = min(PARAM.xmin, PARAMN.xmin) - 1;
PARAM.xmax = max(PARAM.xmax, PARAMN.xmax) + 1;
PARAM.ymin = min(PARAM.ymin, PARAMN.ymin) - 1;
PARAM.ymax = max(PARAM.ymax, PARAMN.ymax) + 1;

Occ = makeOccupancyGrid(ScanRef, PARAM);

it = 0;
% some initialization about
% score
lastbest =[x0 y0 yaw0];
lastgrid = ScanNewTR;
bestfit = sizenew*0.9;
initfit = fitnessGrid2(ScanNewTR, Occ, [x0 y0 yaw0]);
maxfit = initfit;

% and search procedures
incr_x=Opt.scanmatcher.Br(2);
incr_y=Opt.scanmatcher.Br(2);
incr_th=Opt.scanmatcher.Br(1);

lasterror=[];
corr=[];

if DEBUG.gmapping || DEBUG.all
    scrsz = get(0,'ScreenSize');
    opts.fighandle=figure('Position',scrsz,'Renderer','zbuffer','doublebuffer','on');
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    hold all
    opts.plot_r = plot(NaN,NaN,'.r','MarkerSize',6);
    opts.plot_n = plot(NaN,NaN,'.b','MarkerSize',6);
end

while it < niter
    it = it+1;
    
    
    %return if convergence is achieved
    if checkConv(lasterror, corr, 0)
        break;
    end
    
    maxfiti = -1;
    its = 0;
    
    % we loop until we find a better solution or a certain number of
    % iterations is reached
    while maxfiti < 0 && incr_x > 0.0001 && its < niter*2
        
        % we define a set of steps around the axis considering the
        % uncertainty bounds in INCR.
        
        dirs = [  + [incr_x 0 0];
            + [-incr_x 0 0] ;
            + [0 incr_y 0] ;
            + [0 -incr_y 0] ;
            + [0 0 incr_th] ;
            + [0 0 -incr_th] ];
        
        % Evaluate our pose using our fitness function FITNESSGRID which
        % checks a small area around each point using a direct lookup. No
        % association is needed
        
        for j=1:size(dirs,1)
            [fit(j) TRF(j).grid] = fitnessGrid2(ScanNewTR, Occ, dirs(j,:) );
            if fit(j) > maxfit
                maxfit = fit(j);
                maxfiti = j;
            end
        end
        
        % at each iteration we divide by 2 the bounds
        incr_x = incr_x*0.5;
        incr_y = incr_y*0.5;
        incr_th = incr_th*0.5;
        
        its = its+1;
    end
    
    % if a solution better than the current one is found, save the result
    if maxfiti > 0
        corr = [corr; lastbest - dirs(maxfiti,:) ];
        lastbest = dirs(maxfiti,:);
        lastgrid = TRF(maxfiti).grid;
    else
        corr = [corr; 0 0 0];
    end
    
    if DEBUG.gmapping || DEBUG.all
        set(opts.plot_r,'XData',ScanRef(:,1),'YData',ScanRef(:,2));
        set(opts.plot_n,'XData',lastgrid(:,1),'YData',lastgrid(:,2));
        drawnow
    end
    
    
    lasterror = [lasterror abs(log(maxfit/bestfit))-1 ];
    
    % update error
    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
    
end

%% Result
% Returns the minimizing Q = [t R]
t = lastbest(1:2) ;
R = lastbest(3);

NI = it;

end

