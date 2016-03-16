% GA It generates poses
% according to the genetic evolution. At every iteration, a
% number of poses is generated evolving from the previous
% set of poses. The poses that do not have a certain score
% are discarded. The discarded poses are replaced with
% genetic variations of the best poses.

% [1] Mobile robot motion estimation by 2D scan matching with genetic and iterative closest point algorithms
% Martinez, J. L. Gonzalez, J. Morales, J. Mandow, A. Garcia-Cerezo, A. J.
% JOURNAL OF FIELD ROBOTICS
% 2006, VOL 23; NUMBER 1, pages 21-34

% [2] Genetic Algorithm for Simultaneous Localization and Mapping
% Tom Duckett
% Centre for Applied Autonomous Sensor Systems
% Dept. of Technology, Orebro University
% SE-70182 Orebro, Sweden

% [3] Fast Genetic Scan Matching Using
% Corresponding Point Measurements in Mobile Robotics
% Kristijan Lenac1,2 , Enzo Mumolo1 , and Massimiliano Nolich1
% DEEI, University of Trieste, Via Valerio 10, Trieste, Italy
% AIBS Lab, Via del Follatoio 12, Trieste, Italy
function [ R t NI ] = ga( ScanRef, ScanNew, motion )
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

nchr = 50; % N chromosomes. ONLY EVEN NUMBERS
gnsz = 24; % 8-bit representation
chsz = gnsz*3; % Chromosome size (pose)
ScanNew = ScanNew.localCart;
ScanRef = ScanRef.localCart;

sizenew = size(ScanNew,1);

% motion estimation q0
u = motion.con.u;

x0 = u(1);
y0 = u(2);
yaw0 = u(3);
% reference S New in S Ref coordinate system
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

% define the boundaries of search space
bound_x=Opt.scanmatcher.Br(2);
bound_y=Opt.scanmatcher.Br(2);
bound_th=Opt.scanmatcher.Br(1);

bounds_rand = [bound_x bound_y bound_th];

popchr = zeros(nchr,3);

% randomly generate thee initial population
for j = 1:nchr
    tmpx = (rand(1)-0.5)*bound_x*2;
    tmpy = (rand(1)-0.5)*bound_y*2;
    tmpth = (rand(1)-0.5)*bound_th*2;
    
    popchr(j,:) = [tmpx tmpy tmpth];
    
end

% Loop limit as ending criteria
it = 0;
bestfit = sizenew*0.9;


lasterror=[];
corr=[];
lastbest = [x0 y0 yaw0];


if DEBUG.ga || DEBUG.all
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
    % Evaluate our chromosomes using our fitness function FITNESSGRID which
    % checks a small area around each point using a direct lookup. No
    % association is needed
    maxfit = -inf;
    fit = zeros(nchr,1);
    
    selchr = zeros(nchr,2);
    newpopchr = zeros(nchr,3);
    
    
    
    for j = 1:nchr
        [fit(j) TRF(j).grid] = fitnessGrid2(ScanNewTR, Occ, popchr(j,:));
        if fit(j) > maxfit
            maxfit = fit(j);
            maxfiti = j;
        end
    end
    
    corr = [corr; lastbest - popchr(maxfiti,:) ];
    
    if DEBUG.ga || DEBUG.all
        set(opts.plot_r,'XData',ScanRef(:,1),'YData',ScanRef(:,2));
        set(opts.plot_n,'XData',TRF(maxfiti).grid(:,1),'YData',TRF(maxfiti).grid(:,2));
        drawnow
    end
    
    
    lasterror = [lasterror abs(log(maxfit/bestfit))-1 ]
    
    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
    lastbest = popchr(maxfiti,:);
    
    %return if convergence is achieved
    if checkConv(lasterror, corr,0) || isinf(lasterror(end)) || sum(diag(cov(popchr)))<0.1
        break;
    end
    
    
    %% SELECTION: we have to generate a new population of surviving NCHR chromosomes
    % depending on the fitness function. There are different methods to
    % achieve this
    
    survpop = select_duckett(popchr, fit);
    
    
    %% CROSSOVER: the crossover operation merge the two fathers with a
    % probability of 0.7 (common value). The merging is done bitwise,
    % exchanging all genes in the chromosomes sequence after a randomly
    % selected gene. We use a 8-bit representation chromosomes
    for j = 1:nchr/2
        
        % Search for janitors, without reselecting the same couple twice.
        % This ensures diversity. Also try to avoid breeding the same
        % chromosome
        nbreeding = size(survpop,1);
        conds = 1;
        while conds > 0
            
            r1 = floor(rand(1)*nbreeding)+1;
            p1 = survpop((r1),:);
            
            r2 = r1;
            % Draw a new point and check dat it's not too close (or the
            % same) of the first one.
            while r2 == r1
                r2 = floor(rand(1)*nbreeding)+1;
            end
            p2 = survpop((r2),:);
            ddiff = p1 - p2;
            dstc = sqrt(ddiff(1)^2 + ddiff(2)^2) <= 0.000;
            
            if sum(dstc) > 0
                conds = 1;
                continue
            end
            
            % we don't want to select the same pair multiple times.
            conds =  searchPair(selchr,[r1 r2]);
        end
        
        p1 = max(0,floor( ((p1 + bounds_rand))*100000)) ; % We need integer representation. We do a cast assuming a single precision float
        p2 = max(0,floor( ((p2 + bounds_rand))*100000)) ;
        selchr(j,:) = [ r1 r2 ]; % We keep track of the already selected pairs
        
        
        % Not always a crossover operator is applied. In this case we just
        % select the first choice
        % Generate the binary representation in Gray code
        chgc1 = [dec2bin(dec2gc(p1(1)),gnsz) dec2bin(dec2gc(p1(2)),gnsz) dec2bin(dec2gc(p1(3)),gnsz)];
        chgc2 = [dec2bin(dec2gc(p2(1)),gnsz) dec2bin(dec2gc(p2(2)),gnsz) dec2bin(dec2gc(p2(3)),gnsz)];
        
        % Decides whether to use the crossover operator or not
        crossrate = rand(1);
        if crossrate > 0.9
            chnew1 = chgc1;
            chnew2 = chgc2;
        else
            sgene = floor(rand(1) * (chsz-2)) + 1;
            slength = floor(rand(1) * (chsz - (sgene+2) ) ) + 1;
            bounds = sgene+1:sgene+1+slength;
            % Apply the crossover operator and reconverts in Decimal
            chnew1 = [ chgc1(1:sgene) chgc2(bounds)  chgc1(bounds(end)+1:end)];
            chnew2 = [ chgc2(1:sgene) chgc1(bounds)  chgc2(bounds(end)+1:end)];
        end
        
        
        %% MUTATION: Once we have our new chromosome, we might apply a mutation operator
        % with very low rate. Mutation might have bad effects on diversity.
        % An application of mutation usually brings to significant changes
        % and improbable solutions. IMPROVE to avoid this
        mutationrate = rand(1);
        if mutationrate < 0.001
            % we select one bit and flip it
            sgene = floor(rand(1) * chsz) + 1;
            b = num2str(bitxor( sscanf(chnew1(sgene),'%i') ,1));
            chnew1(sgene) = b;
        end
        mutationrate = rand(1);
        if mutationrate < 0.001
            % we select one bit and flip it
            sgene = floor(rand(1) * chsz) + 1;
            b = num2str(bitxor( sscanf(chnew2(sgene),'%i') ,1));
            chnew2(sgene) = b;
        end
        
        nch1 = [ gc2dec(bin2dec(chnew1(1:gnsz))) /100000 , ...
            gc2dec(bin2dec(chnew1(gnsz+1:gnsz*2)))/100000, ...
            gc2dec(bin2dec(chnew1(gnsz*2+1:gnsz*3)))/100000]-bounds_rand;
        nch2 = [gc2dec(bin2dec(chnew2(1:gnsz)))/100000, ...
            gc2dec(bin2dec(chnew2(gnsz+1:gnsz*2)))/100000, ...
            gc2dec(bin2dec(chnew2(gnsz*2+1:gnsz*3)))/100000]-bounds_rand;
        
        newpopchr( ((j-1)*2) +1,:) = nch1;
        newpopchr( ((j-1)*2) +2,:) = nch2;
        
    end
    
    popchr = single(newpopchr);
end

%% result
t = popchr(maxfiti,1:2);
R = popchr(maxfiti,3);
NI = it;
end

