%FMTSM Spectral registration takes proﬁt of the fact that image translation in
% the space domain manifest itself as a phase shift in the
% frequency domain. Hence, given the Fourier transform
% of two translated but not rotated images, corresponding
% to the Sref and Snew scans, it is possible to compute
% their phase shift using the Phase Only Matched Filter
% (POMF). Next, the inverse Fourier transform of the
% phase difference is a Dirac delta signal centred in the
% corresponding image translation. Computing its peak,
% means solving for the image translation. On the other hand,
% a rotation applied to an image in the space
% domain is manifested as a rotation of the Fourier spectra.
% However, the rotation is manifested as a signal shift in
% the polar representation of the Fourier Magnitude, which
% may again be computed using a POMF over the polar re-
% sampled Fourier transform of the images and computing
% the peak of the corresponding Dirac Delta signal. It is
% worth noting, that it is necessary to ﬁrst solve for the
% rotation and next, solve for the translation.

% [25] A. B. Heiko Bulow, Max Pﬁngsthorn, “Using Robust Spectral Regis-
% tration for Scan Matching of Sonar Range Data ,” in The 7th IFAC
% Symposium on Intelligent Autonomous Vehicles, IAV 2010. IEEE, 2010.
function [ R t NI] = fmtsm( ScanRef, ScanNew, motion )
% In:
%           ScanRef: localCart[2xN], points cartesian
%           ScanRef: localCart[2xN], points cartesian
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations





global DEBUG Opt;

%Calculate the motion parameters
u = motion.con.u;
pose(3) = 0;


x0 = u(1);
y0 = u(2);
yaw0 = u(3);

x=x0;
y=y0;
yaw=yaw0;
NI = 1;
borders = 0;

rTn = [cos(yaw) -sin(yaw) x; sin(yaw) cos(yaw) y; 0 0 1];
Snew_Or =ScanNew.localCart;
magnThreshold = 10;

%  S New referenced to S Ref coordinate system
for z = 1:size(ScanNew.localCart,1)
    ScanNew.localCart(z,1:3) = (rTn*[ScanNew.localCart(z,1:2) 1]')' ;
    th = normAngle(ScanNew.localPolar(z,1) + yaw);
    ro = ScanNew.localPolar(z,2);
    rc = cos(th)*ro + x;
    rs = sin(th)*ro + y;
    
    ScanNew.localPolar(z,2) = sqrt(rc^2 + rs^2);
    ScanNew.localPolar(z,1) = atan2(rs,rc);
end

ScanNew.localCart(:,3) =  pose(3);


% Make the occupancy grid as big as to contain both scans. The occupancy
% grid is also our binary image that will be processed through the fourier
% mellin transform. Other methods might be used which take into
% consideration the sensor uncertainty and sensor type.
[PARAM.xmin PARAM.xmax PARAM.ymin PARAM.ymax] = minmaxScan(ScanRef.localCart);
[PARAMN.xmin PARAMN.xmax PARAMN.ymin PARAMN.ymax] = minmaxScan(ScanNew.localCart);
PARAM.L = Opt.map.resolution;  %Occupancy resolution
PARAM.type = 'binary';
PARAM.xmin = min(PARAM.xmin, PARAMN.xmin) - borders;
PARAM.xmax = max(PARAM.xmax, PARAMN.xmax) + borders;
PARAM.ymin = min(PARAM.ymin, PARAMN.ymin) - borders;
PARAM.ymax = max(PARAM.ymax, PARAMN.ymax) + borders;

OccRef = makeOccupancyGrid(ScanRef.localCart, PARAM);
OccNew = makeOccupancyGrid(ScanNew.localCart, PARAM);

DEBUGALL = 0;

Sref = ScanRef.localCart;
bmw = blackman(128);


t_res(1) = Opt.scanmatcher.Br(2) / 128;
t_res(2) = Opt.scanmatcher.Br(2) / 128;

gridsize = 128;

%% Reference Scan (S REF)

%OccRef.grid = OccRef.grid .* hann2d(imagesize(1),imagesize(2));
%OccNew.grid = OccNew.grid .* hann2d(imagesize(1),imagesize(2));


% 2D FFT of S Ref
fftImage1 = fftshift(fft2(OccRef.grid));
magn1 = (abs((fftImage1)));
phase1 = angle(fftImage1);

if (DEBUG.all || DEBUG.fmtsm) && DEBUGALL
    % plot the image, its magnitude and its phase
    figure;
    subplot(1,3,1), imshow(OccRef.grid, [0 1]);
    subplot(1,3,2), imshow(magn1,[0 15]);
    subplot(1,3,3), imshow(phase1,[-pi/2 pi/2]);
end

% resample into polar coordinates the magnitude so to have a FMT descriptor
% in MAGNPOLAR1 and filter it
magnPolar1 = resamplePolar(magn1,gridsize);
magnPolar1(find(magnPolar1<magnThreshold)) = 0;

% apply a blackman window and calculate the phase of the FMT descriptor
for kk = 1:size(magnPolar1,2)
    magnPolar1(:,kk) = magnPolar1(:,kk).*bmw;
end

% apply a fft and calculate magntude and phase of the FMT descriptor
magnPolarT1 = fftshift(fft2(magnPolar1));
phasePolar1 = angle(magnPolarT1);

if (DEBUG.all || DEBUG.fmtsm) && DEBUGALL
    % plot the polar resampled magnitude (FMT descriptor),
    % its magnitude (of the FMT descriptor) and its phase
    figure;
    subplot(1,3,1), imshow( ((magnPolar1 )),[0 25]);
    subplot(1,3,2), imshow( (abs(imag(magnPolarT1) )),[0 25]);
    subplot(1,3,3), imshow(phasePolar1,[-pi/2 pi/2]);
end

%% New Scan (S NEW)

% 2D FFT of S New
fftImage2 = fftshift(fft2(OccNew.grid));
magn2 = (abs((fftImage2)));
phase2 = angle(fftImage2);


if (DEBUG.all || DEBUG.fmtsm) && DEBUGALL
    % plot the image, its magnitude and its phase
    figure;
    subplot(1,3,1), imshow(OccNew.grid, [0 1]);
    subplot(1,3,2), imshow(magn2,[0 15]);
    subplot(1,3,3), imshow(phase2,[-pi/2 pi/2]);
end

% resample into polar coordinates the magnitude so to have a FMT descriptor
% in MAGNPOLAR1 and filter it
magnPolar2 = resamplePolar(magn2,gridsize);
magnPolar2(find(magnPolar2<magnThreshold)) = 0;

% apply a blackman window and calculate the phase of the FMT descriptor
for kk = 1:size(magnPolar2,2)
    magnPolar2(:,kk) = magnPolar2(:,kk).*bmw;
end

% apply a fft and calculate magntude and phase of the FMT descriptor
magnPolarT2 = fftshift(fft2(magnPolar2));
phasePolar2 = angle(magnPolarT2);

if (DEBUG.all || DEBUG.fmtsm) && DEBUGALL
    % plot the polar resampled magnitude (FMT descriptor),
    % its magnitude (of the FMT descriptor) and its phase
    figure;
    subplot(1,3,1), imshow( ((magnPolar2 )),[0 25]);
    subplot(1,3,2), imshow( (abs( imag(magnPolarT2) )),[0 25]);
    subplot(1,3,3), imshow(phasePolar2,[-pi/2 pi/2]);
end

%% Rotational Hypotheses
% Do the inverse of phase difference and calculate its inverse fft
invimg = ifft2(exp( -(phasePolar2 - phasePolar1 )*1j ) );
% Optional: ZERO FREQUENCY invimg(1) = 0;
img = ((ifftshift(invimg)));

% Calculate the magnitude of the inverse and smooth it to have clearer
% maxima
imgs = sum(abs((img)).^2);
imgs = smooth(imgs,5);


if (DEBUG.all || DEBUG.fmtsm) && DEBUGALL
    % print the 1D containing the rotational displacement info
    figure;
    plot( imgs  ); colormap jet;
    grid on;
end

% extract the maxima which represent the number of rotatioanl hypotheses we
% want to investigate and convert to radians.
[ maxA ] = findLocalMaxima(imgs',2);
maxA = deg2rad(floor(maxA*(180/(gridsize-1)) - 90));

% some initialization for the hypotheses
besti = -1;
bestval = -inf;
bestr = 0;
bestx=0;
besty=0;

% For each rotataional hypothesis
for i = 1:size(maxA,2)
    
    % rotate the image according to the hypothesis
    BestRot = maxA(i);
    ImgRot = imrotate(OccNew.grid, rad2deg(BestRot),'nearest','crop');
    
    % apply a fft to extract magnitude and phase
    fftImage2R = fftshift(fft2(ImgRot));
    magn2t = (abs(fftImage2R));
    phase2t = angle(fftImage2R);
    
    if (DEBUG.all || DEBUG.fmtsm) && DEBUGALL
        % plot the rotated image, its magnitude and phase
        figure;
        subplot(1,3,1), imshow(ImgRot);
        subplot(1,3,2), imshow(magn2t,[0 15]);
        subplot(1,3,3), imshow(phase2t,[-pi/2 pi/2]);
    end
    
    % inverse of difference phase with the phase of S Ref
    invimg = ifft2(exp( -(phase2t - phase1)*1j ) );
    % Optional: ZERO FREQUENCY invimg(1) = 0;
    img = abs((ifftshift(invimg)));
    
    
    if (DEBUG.all || DEBUG.fmtsm) && 1
        figure;
        surf(img); shading interp; colormap jet;
        fmt_res = figure;
    end
    
    %% Translational hypotheses
    % Extract the translational hypotheses from the Dirac pulse
    THypothesis = findLocalMaxima2(img,20);
    
    
    maxX = THypothesis(1,:);
    maxY = THypothesis(2,:);
    
    
    % we search for the X and Y correction indipendently
    for k = 1:size(maxX,2)
        
        y = (maxX(k)-floor(size(img,1)/2)) * t_res(1);
        for jj = 1:size(maxY,2)
            % Optional, only consider maxima greater than a certain value
            %         if(maxT < maxT0*0.8)
            %             break;
            %         end
            
            
            x = (maxY(jj)-floor(size(img,2)/2)) * t_res(2);
            
            % Generate the hypothesis, including the current rotation and
            % weight it using a fitness function
            thyp(1:2) = [x y];
            thyp(3) = BestRot;
            
            
            [score lastgrid] = fitnessGrid2(Snew_Or, OccRef, [x0 y0 yaw0] + [thyp(1) thyp(2) thyp(3)]);
            
            % Save the best value
            if(score > bestval)
                bestval = score;
                besti = k;
                bestx = thyp(1);
                besty = thyp(2);
                bestr = thyp(3);
                
                if DEBUG.fmtsm || DEBUG.all
                    % display the new best match found
                    figure(fmt_res);
                    hold on
                    axis equal
                    grid on
                    displayPoints(Sref,'g',0);
                    displayPoints(lastgrid,'r',0);
                end
            end
            
        end
    end
end

%% Result
if DEBUG.fmtsm || DEBUG.all
    % Display the result
    figure('Name','FMT RES'); hold on
    title( ['\theta = ' num2str(bestr) ', x=' num2str(bestx) ', y=' num2str(besty) ', score= ' num2str(bestval) ]);
    displayPoints(Sref,'g',0);
    displayPoints(applyTransform2Scan2D(Snew_Or, bestr+yaw0, [x0 y0] + [bestx besty]),'r',0);
    axis equal
end

t = [bestx besty] ;
R = (bestr);

if DEBUG.fmtsm || DEBUG.all
    close(fmt_res);
end

end
