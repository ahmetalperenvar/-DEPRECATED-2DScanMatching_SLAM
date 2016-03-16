% MONTECARLO: The algorithm works similarly to
% VASCO but instead of searching over the axis of the
% ellipse, it randomly generats a set of possible poses
% inside the uncertainty ellipse and then assigns a score
% using a grid.
function [ R t NI ] = montecarlo( ScanRef, ScanNew, motion )
% In:
%           ScanRef: localCart[2xN], points cartesian
%           ScanRef: localCart[2xN], points cartesian
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations


global DEBUG Opt

% maximum iterations
niter = Opt.scanmatcher.iterations;
% point sets
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


% Define the searches bound
bound_x=Opt.scanmatcher.Br(2);
bound_y=Opt.scanmatcher.Br(2);
bound_th=Opt.scanmatcher.Br(1);


it = 0;


lastbest =[0 0 0];


% Initial score
bestfit = sizenew*2;
initfit = fitnessGrid2(ScanNewTR, Occ, [0 0 0]);
maxfit = initfit;


% Some initialization
lasterror=[];
corr=[];
xtot= x0;
ytot= y0;
yawtot= yaw0;

%How many samples at each iteration
NSamples = 50;

if DEBUG.montecarlo || DEBUG.all
    scrsz = get(0,'ScreenSize');
    opts.fighandle=figure('Position',scrsz,'Renderer','zbuffer','doublebuffer','on');
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    hold all
    opts.plot_r = plot(NaN,NaN,'.r','MarkerSize',6);
    opts.plot_n = plot(NaN,NaN,'.b','MarkerSize',6);
end

bound_d = 1;

while it < niter
    it = it+1;
    
    
    %return if convergence is achieved
    if checkConv(lasterror, corr,0)
        break;
    end
    
    maxfiti = -1;
    ns=zeros(NSamples,3);
    
    
    for j=1:NSamples
        ns(j,:) = [stdErr(0,bound_x/bound_d) stdErr(0,bound_y/bound_d) stdErr(0,bound_th/(bound_d*2))];
        res = frameRef([xtot ytot yawtot]', [ns(j,1:2) ns(j,3)]' )';
        ns(j,1:3) = res(1:3);
    end
    
    % Evaluate our pose using our fitness function FITNESSGRID which
    % checks a small area around each point using a direct lookup. No
    % association is needed
    for j=1:size(ns,1)
        [fit(j) TRF(j).grid] = fitnessGrid2(ScanNew, Occ,  ns(j,:) );
        if fit(j) > maxfit
            maxfit = fit(j);
            maxfiti = j;
        end
    end
    
    % If we found a better correction we save the last pose
    if maxfiti > 0
        corr = [corr; lastbest - ns(maxfiti,:) ];
        lastbest = ns(maxfiti,:);
        lastgrid = TRF(maxfiti).grid;
        bound_d = bound_d+0.5;
        
        
        if DEBUG.montecarlo || DEBUG.all
            set(opts.plot_r,'XData',ScanRef(:,1),'YData',ScanRef(:,2));
            set(opts.plot_n,'XData',lastgrid(:,1),'YData',lastgrid(:,2));
            drawnow
        end
        
        % Last correction info
        xtot = lastbest(1);
        ytot = lastbest(2);
        yawtot = lastbest(3);
        
    else
        corr = [corr; 0 0 0];
    end
    
    
    % Error information
    err_l = exp(-log(maxfit/bestfit))-1;
    lasterror = [lasterror err_l ];
    
    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
end

%% Result
% Returns the minimizing Q = [t R]
t =  [ xtot(1) ytot(1) ] - [x0 y0 ];
R = normAngle(yawtot(1)-yaw0);

NI = it;

end

