% The idea of NDT is to model
% a scan distribution as a grid of normal distributions.
% First, a likelihood grid is created where each point in
% the reference scan is inserted. The grid behaves as a
% piecewise approximation of the likelihood field. Then
% the negative log likelihood function of observing the
% new scan, Snew , given a hypothetical roto-translation is
% defined. The function is defined evaluating the likelihood
% of observing each single point of the Snew scan falling in
% a particular grid cell (depending on the roto-translation
% hypothesis). The log likelihood is next simplified and
% converted into a score function which is next minimized
% using the Newton’s method.

% Antoni BURGUERA, Yolanda GONZÁLEZ, Gabriel OLIVER
% The sNDT: A Grid-Based Likelihood Field Approach to Robust and Accurate Sonar Scan Matching Localization.
% Technical Report A-06-2007
% Departament de Matemàtiques i Informàtica (UIB)

function [ R t NI] = NDT( SRef, SNew, motion )

% In:
%           Ai: localCart[2xN], points cartesian,
%           Bi: localCart[2xN], points cartesian,
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations


global DEBUG Opt

itMax = Opt.scanmatcher.iterations;

ScanNew = SNew.localCart;
ScanRef = SRef.localCart;

sizenew = size(ScanNew,1);

%Motion estimation q0
u = motion.con.u;

x0 = u(1);
y0 = u(2);
yaw0 = u(3);

originalp = [x0 y0 yaw0];
p = originalp;


% Find the bounds
[PARAM.xmin PARAM.xmax PARAM.ymin PARAM.ymax] = minmaxScan(ScanRef);
PARAM.L = Opt.map.resolution;  %Occupancy resolution

L = PARAM.L;
xmin = PARAM.xmin;
xmax = PARAM.xmax;
ymin = PARAM.ymin;
ymax = PARAM.ymax;

xwide = xmax - xmin;
ywide = ymax - ymin;

NX = floor(xwide / L);
NY = floor(ywide / L);


%generate the grids to fit both scans
[ subGrids] = makeLocalOccupancyGrid(ScanRef, PARAM);

lasterror = [];
corr=[];


%occGrid1-4 represents the 4 four different grids built over the map with
%L/2 shift on 3 directions. See the function for details. We are going to
%use Newton's algorithm to minimize the score function which uses the sum
%of the four different PDF defined for each cell.
merr=0;

if DEBUG.NDT || DEBUG.all
    figure
    hold on
    axis equal
    hr = plot(ScanRef(:,1),ScanRef(:,2),'.k');
    hs = plot(NaN,NaN,'.b');
    title(['err = ' num2str(1/merr)]);
end

for i = 1:itMax*2
    
    %return if convergence is achieved
    if checkConv(lasterror, corr)
        break;
    end
    
    x = p(1);
    y = p(2);
    yaw = p(3);
    cosy = cos(yaw);
    siny = sin(yaw);
    
    
    % initialize the gradient and Hessian for this iteration
    grad = zeros(1,3);
    H = zeros(3,3);
    
    
    
    merr = 0;
    totgrds = 0;
    
    curscan = [];
    
    % loop through all the points in  S NEW and find the cell it falls in
    % in the 4 grids.
    for j = 1:sizenew
        pn = ScanNew(j,1:2);
        ngrds = 0;
        
        for k = 1:size(subGrids,2)
            
            ogrid = subGrids(k);
            
            gradS = zeros(1,3);
            Hps = zeros(3,3);
            %transform the point using the latest estimation
            v(1) = x + cosy*pn(1) - siny*pn(2);
            v(2) = y + siny*pn(1) + cosy*pn(2);
            
            curscan = [curscan [v(1); v(2) ] ];
            
            %find in which cell of the grid it falls in
            [ix iy] = getCoordinatesSubgrid(v(1),v(2),k,PARAM);
            
            %We need to be inside the grid to have overlap
            if ix < 1 || ix > NX || iy < 1 || iy > NY
                continue;
            else
                %retrieve mean and covariance and of that cell and find the
                %gradient of the likelihood function. The likelihood function
                %is a pdf defined by mean and covariance created by the
                %occupancy grid. It is used to find the likelihood of a point
                %projected of being in a particular area of the grid. We need
                %to derive this function to find the new estimation
                m = ogrid.grid(ix,iy).mean;
                cov = ogrid.grid(ix,iy).cov;
            end
            
            if m == 0
                continue
            end
            
            % if the matrix is singular, we need a backup plan
            [~, cov] = checkSingular(cov,1);
            
            invcov = inv(cov);
            
            %Gradient of the transformation function
            dv = [ 1, 0, -pn(1)*siny-cosy*pn(2);...
                0, 1, pn(1)*cosy-siny*pn(2)];
            
            
            %difference with mean
            dnm = (v - m)';
            
            % Calculate the Hessian of the likelihood function
            vH = [ -pn(1)*cosy+siny*pn(2);
                -pn(1)*siny-cosy*pn(2)   ];
            
            covdnm = (dnm')*invcov;
            exppart = exp( -(covdnm*dnm)/2 )  ;
            
            % Gradient vector
            gradt = (covdnm * dv * exppart  );
            ngrds = ngrds+1;
            
            
            dnmdv1 = covdnm*dv(:,1);
            dnmdv2 = covdnm*dv(:,2);
            dnmdv3 = covdnm*dv(:,3);
            
            dnmtdv1 = dv(:,1)'*invcov;
            dnmtdv2 = dv(:,2)'*invcov;
            dnmtdv3 = dv(:,3)'*invcov;
            
            % Score function as error
            merr = merr - exppart;
            
            % Hessian of the function
            Hp =  [  ( (-dnmdv1)*dnmdv1-dnmtdv1*dv(:,1) )*-exppart,...% D² xx
                ( (-dnmdv1)*dnmdv2-dnmtdv2*dv(:,1)  )*-exppart,... % D² xy
                ( (-dnmdv1)*dnmdv3-dnmtdv3*dv(:,1) )*-exppart;... % D² xTHETA
                ...
                0,...% D² yx
                ( (-dnmdv2)*dnmdv2-dnmtdv2*dv(:,2) )*-exppart,... % D² yy
                ( (-dnmdv2)*dnmdv3-dnmtdv3*dv(:,2) )*-exppart;... % D² yTHETA
                ...
                0,...% D² x THETA
                0,... % D² yTHETA
                ( (-dnmdv3)*dnmdv3-dnmtdv3*dv(:,3)-(covdnm*vH) )*-exppart...
                % D² THETA THETA. The second derivative is defined only in this point
                ];
            %H is symmetric, let's fill the remaining part
            Hp(2,1) = Hp(1,2);
            Hp(3,1) = Hp(1,3);
            Hp(3,2) = Hp(2,3);
            
            gradS = gradS + gradt;
            Hps = Hps + Hp;
            
            
        end

        H = H + Hps;
        grad = grad + gradS;

    end
    
    % Once we have the Hessian and Gradient of the function we calculate
    % the new translation
    invh = inv(H);
    if( sum(sum(isinf(invh))) == 0)
        dt = -invh*(grad');
    else
        % the Hessian inversion has failed
        break;
    end

    if ( DEBUG.NDT || DEBUG.all ) && size(curscan,1)>0
        size(curscan)
        set(hs,'XData',curscan(1,:),'YData',curscan(2,:));
        drawnow
    end
    
    p = p + dt'; 
    p(3) = normAngle(p(3));
    
    % Maintain last errors to check convergence
    lasterror = [lasterror 1/merr ];
    corr = [corr; dt(1) dt(2) dt(3)];

    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
end

%Final result in quaternion
R = normAngle(p(3)-originalp(3));

t = [p(1)-originalp(1) p(2)-originalp(2) ];

NI = i;

end


