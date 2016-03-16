
%HOUGH The discrete Hough
% Transform (HT ) of both scans Sref and Snew is com-
% puted. For each point in a scan set, HT maps a set
% of possible regular features passing through that point.
% The features are parametrized, as a set of (ρ,θ) values.
% The transform returns a 2D matrix of accumulators
% which have higher values when a possible feature passes
% through more points in the scan set. This domain is
% invariant to the translation, but not to the rotation. Cal-
% culating the magnitude of the HT , it is possible to have
% a 1D signal carrying rotational information. Correlating
% the two signals of two scan sets, it is possible to extract
% a series of rotational correction hypotheses. To ﬁnd the
% translation a different procedure is applied. We begin
% by rotating Snew with one of the rotational hypotheses
% previously found and correlating its HT with the one
% of Sref . The correlation maxima will identify a set of
% column indexes C (or directions of correction) which we
% will use to ﬁnd the translation hypotheses. A discretized
% space of translational hypotheses with resolution νd and
% bounded at [-δd ,+δd ]is generated. Each cell represents
% a hypothesis and it is ﬁlled with the sum (for each
% direction) of the cross correlation value (related with
% the cell hypothesis) of the C indexed columns extracted
% from the HT of Sref and the HT of Snew rotated.
% From the transational buffer a series of x, y hypotheses
% is extracted.

% A. Censi, L. Iocchi, G. Grisetti  -
% Scan Matching in the Hough Domain (PDF), presented at ICRA 2005.

function [ R t NI] = houghSM( ScanRef, ScanNew, motion )

% In:
%           ScanRef: localCart[2xN], points cartesian
%           ScanRef: localCart[2xN], points cartesian
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations


global Opt

%Calculate the motion parameters
u = motion.con.u;
lrangle = u(2);

x0 = u(1);
y0 = u(2);
yaw0 = u(3);

x=x0;
y=y0;
yaw=yaw0;

borders = 2;
NI = 0;
rTn = [cos(yaw) -sin(yaw) x; sin(yaw) cos(yaw) y; 0 0 1];

% B referenced to A coordinate system
for i = 1:size(ScanNew.localCart,1)
    ScanNew.localCart(i,1:3) = (rTn*[ScanNew.localCart(i,1:2) 1]')' ;
    th = normAngle(ScanNew.localPolar(i,1) + yaw);
    ro = ScanNew.localPolar(i,2);
    rc = cos(th)*ro + x;
    rs = sin(th)*ro + y;
    
    ScanNew.localPolar(i,2) = sqrt(rc^2 + rs^2);
    ScanNew.localPolar(i,1) = atan2(rs,rc);
end

% Make the occupancy grid as big as to contain both scans
[PARAM.xmin PARAM.xmax PARAM.ymin PARAM.ymax] = minmaxScan(ScanRef.localCart);
[PARAMN.xmin PARAMN.xmax PARAMN.ymin PARAMN.ymax] = minmaxScan(ScanNew.localCart);
PARAM.L = Opt.map.resolution;  %Occupancy resolution
PARAM.type = 'binary';
PARAM.xmin = min(PARAM.xmin, PARAMN.xmin) - borders;
PARAM.xmax = max(PARAM.xmax, PARAMN.xmax) + borders;
PARAM.ymin = min(PARAM.ymin, PARAMN.ymin) - borders;
PARAM.ymax = max(PARAM.ymax, PARAMN.ymax) + borders;

% Some parameter for the algorithm
maxPhiHypotheses=4;
rhoScale=Opt.map.resolution;
thetaScale=1;
rhoCells = 200;
thetaCells = round(360/thetaScale);
maxTHypotheses = 6;
phiFilterSigma = 15;
rhoFilterSigma = 4;
maxRhoCorrelations = 8;
maxTBound = Opt.scanmatcher.Br(2);


global DEBUG

Sref = ScanRef.localCart;
if(size(Sref,2)>2)
    Sref(:,3) = [];
end
Snew = ScanNew.localCart;
if(size(Snew,2)>2)
    Snew(:,3) = [];
end

Sref = Sref';
Snew = Snew';

HoughRef = computeHoughTransform(Sref, thetaCells, rhoCells, rhoScale);
HoughNew = computeHoughTransform(Snew, thetaCells, rhoCells, rhoScale);

if DEBUG.houghSM || DEBUG.all
    % show the Hough transform
    figure;
    subplot(4,2,3); imshow(HoughRef, [ 0 14]);
    subplot(4,2,4); imshow(HoughNew, [ 0 14]);
    colormap(hot);
    
end

% Compute the discrete hough transform using the algorithm provided by the
% author
HoughSpectrumRef = computeHoughSpectrum(HoughRef,true);
HoughSpectrumNew = computeHoughSpectrum(HoughNew,true);

% and calculate the cross correlation which is shown if on debug
ccHough = circularCrossCorrelation(HoughSpectrumNew,HoughSpectrumRef,true);

if DEBUG.houghSM || DEBUG.all
    subplot(4,2,5); plot(1:size(HoughSpectrumRef',1),HoughSpectrumRef,'-r', 1:size(HoughSpectrumNew',1),HoughSpectrumNew,'-b');grid on;
    subplot(4,2,6); plot(ccHough);grid on;
end

%% Rotation

% To find an estimate of the rotation, we do a
% circular cross-correlation of the two spectra
crossCorrelation = circularCrossCorrelation(HoughSpectrumNew, HoughSpectrumRef, true);

% We then do simple low-pass filtering to eliminate noise
crossCorrelationFiltered = simpleLowPassFilterCircular(crossCorrelation, phiFilterSigma);

% We then extract the maxima of the filtered cross correlation:
% these are our hypotheses for the rotation
phiIndexes = findLocalMaximaCircular(crossCorrelationFiltered, maxPhiHypotheses);
phiHypotheses = normAngle(phiIndexes * 2 * pi / thetaCells);

tvec=[];

%% Translation

% Now we find a translation for each rotation hypothesis
for i=1:size(phiHypotheses,2)
    phi=(phiHypotheses(i));
    
    
    % Assuming phi, we rotate the second spectrum
    % radians->cells
    phiCell = phi / (2*pi) * thetaCells;
    % circular rotation
    HoughSpectrumNewR = circularRotation(HoughSpectrumNew, phiCell);
    
    % We have to choose the columns (=which directions) of the HT to correlate
    % the best way to choose is to use the maxima of the spectrum;
    % therefore (to suppress noise) we multiply the two spectra, filter
    % and get the maxima
    product = HoughSpectrumRef .* HoughSpectrumNewR;
    productFiltered = simpleLowPassFilterCircular(product, phiFilterSigma);
    
    corrIndexes = findLocalMaxima(productFiltered(1:floor(thetaCells/2)), maxRhoCorrelations);
    
    if (size(corrIndexes,2)==1)
        corrIndexes(2)  =  round(corrIndexes(1)+thetaCells/4);
    end
    
    cells = round(maxTBound * (rhoCells/rhoScale));
    
    % the buffer of translational hypotheses
    tbuffer = zeros(cells*2+1,cells*2+1);
    
    % For each direction, fill the buffer with the correlation of the
    % columns defined before
    for j=1:size(corrIndexes,2)
        index=corrIndexes(j);
        direction=index*2*pi/thetaCells;
        
        % We extract the two columns of the HT
        col1 = HoughRef(index,:);
        % we must "rotate" by phi
        index2 = min(round(mod(index+phiCell, thetaCells))+1,size(HoughNew,1) );
        col2 = HoughNew(index2,:);
        
        % We do a cross correlation between the two columns
        crossCorrelationRange = cells;
        crossCorrelation = xcorr(col2,col1,crossCorrelationRange);
        
        % Now for each element in the T buffer,
        % we increase the value by this observation
        for a=1:size(tbuffer,1)
            for b=1:size(tbuffer,2)
                % Which value of the translation T is associated to this cell?
                T = [ (a-cells) (b-cells) ]' * rhoScale / rhoCells;
                
                % We project T on the correlation direction
                pT = [cos(direction) sin(direction)] * T;
                
                % We transform the projection to cells of the correlation
                % buffer
                pTcell = round(pT * rhoCells / rhoScale + cells);
                
                if (pTcell>=1) && (pTcell<=size(crossCorrelation,2))
                    value = crossCorrelation(pTcell);
                    tbuffer(a,b)=tbuffer(a,b)+value;
                end
            end
        end
    end
    
    
    % Now the buffer is complete, to find the T we extract the maxima
    % (for simplicity, we use only one maximum)
    
    tbufferFiltered = simpleLowPassFilter2(tbuffer, rhoFilterSigma);
    Tcells = findLocalMaxima2(tbufferFiltered, maxTHypotheses);
    num=size(Tcells,2);
    % Convert back to world-coordinates
    THypotheses = (Tcells-repmat([cells cells]',1,num)) * rhoScale / rhoCells;
    
    % Saving debug info into main structure
    THypotheses(3,:) = repmat(phi,1,size(THypotheses,2));
    tvec = [tvec THypotheses];
    
end

% some initialization for the best correction search
maxf = inf;
maxi=1;


% we use nearest neighbour to find the best correction among those
% extracted from the Hough domain
for s=1:size(tvec,2)
    
    
    rot = -tvec(3,s);
    t = -[tvec(1:2,s)];
    R = e2q([0 0 rot]);
    corrs=[];
    rTn = [cos(rot) -sin(rot) t(1); sin(rot) cos(rot) t(2); 0 0 1];
    
    % B referenced to A coordinate system
    for i = 1:size(Snew,2)
        corrs(:,i) = (rTn*[Snew(1:2,i); 1]) ;
    end
    corrs(3,:) = [];
    % returns the nearest neighbour error
    [~, err] = dsearchn(corrs', Sref');
    err = mean(err);
    if err < maxf
        maxf = err;
        maxi = s;
        %maxf = j;
    end
end



%% Result

t = -tvec(1:2,maxi)';
R=-tvec(3,maxi);

if DEBUG.houghSM || DEBUG.all
    % fisplay the final result
    figure;
    hold on
    axis equal
    grid on
    displayPoints(Sref','g',0);
    displayPoints(Snew','b',0);
    displayPoints(applyTransform2Scan2D(Snew', R, [t(1) t(2)]),'r',0);
end

end



