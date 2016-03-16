close all; clear all;

global PARAM

% pIC parameters RETOCAR!!!
PARAM.deviation.sensor              = [0.05 deg2rad(1.5)] % std deviation [angular(rad) range(m)] [+/-1.5º +/-5cm]
%PARAM.deviation.sensor              = [0 0] % std deviation [angular(rad) range(m)] [+/-1.5º +/-5cm]
PARAM.deviation.motion              = [0.1 0.1 deg2rad(10)] % motion std deviation [x(m) y(m) yaw(rad)]
PARAM.step                          = 2;
PARAM.confidence                    = 0.95;
PARAM.maxDistanceInterpolation      = 0.5;
PARAM.filter                        = 1;
PARAM.projectionFilter              = 0; %!! test with 0
PARAM.associationError              = 0.1;
PARAM.maxIterations                 = 250;
PARAM.errorRatio                    = 0.0001;
PARAM.error                         = [0.0001 0.0001 0.0001];
PARAM.nIterationSmoothConvergence   = 2;

LastScanK = load('scan5.mat');
LastScanK1 = load('scan6.mat');%LastScanK; %load('scan6.mat');

motion.estimation = [-0.2 0.6 deg2rad(5)];
%motion.estimation = [-0.2 1 deg2rad(-5)]; % Real data. No noise added

threshold = 90;
maxRange = 18;


% World configuration
XWorld = 16; YWorld = 8;

% Sensor configuration
sensor.range = 10;
sensor.rangeDev = PARAM.deviation.sensor(2);
sensor.sector = deg2rad(360);
sensor.thetaStep = deg2rad(1.8);
sensor.thetaDev = PARAM.deviation.sensor(1);

positionK = [8,4,deg2rad(30)];
figure
[DataK,TMP1,TMP2] = SimulateProfiler(LastScanK.scan,threshold,maxRange);
scanK = Preprocess(DataK);

 
% % Main While 
errorK1 = 1000000;
nConverged = 0;
nIteration = 0;
% 
positionK1 = [10,5,deg2rad(60)];
%positionK1 = [9,4,0];
figure
[DataK1,TMP1,TMP2] = SimulateProfiler(LastScanK1.scan,threshold,maxRange);
scanK1 = Preprocess(DataK1);

% Struct for motion
% Estimation with noise
disp('motion.estimation');
%motion.estimation = positionK1-positionK;
motion.estimation
% inici = motion.estimation;
% motion.estimation(1) = motion.estimation(1)+normrnd(0,PARAM.deviation.motion(1));
% motion.estimation(2) = motion.estimation(2)+normrnd(0,PARAM.deviation.motion(2));
% motion.estimation(3) = motion.estimation(3)+normrnd(0,PARAM.deviation.motion(3));
% motion.estimation(1) = motion.estimation(1) - 0.5;
% motion.estimation(2) = motion.estimation(2) - 0.5;
% motion.estimation(3) = motion.estimation(3) - deg2rad(10);
% Covariance

handle = figure;
hold on
hscanK = plot(scanK.cart(1,:),scanK.cart(2,:),'b.');
hscanK1 = plot(scanK1.cart(1,:),scanK1.cart(2,:),'r.');
hscanSol = plot(scanK1.cart(1,:),scanK1.cart(2,:),'g.');
legend([hscanK,hscanK1,hscanSol],'scan k','skan k+1','solution',1);

motion.P = [PARAM.deviation.motion(1)^2 0 0; 0 PARAM.deviation.motion(2)^2 0; 0 0 PARAM.deviation.motion(3)^2];

disp('noisy motion.estimation');
motion.estimation
out = 0;
while (nIteration < PARAM.maxIterations && out == 0)
    [resEStep,assoc] = EStep(scanK, scanK1, motion);

    [resMStep,solution,errorK1,nConverged] = MStep(assoc,motion,errorK1,nConverged);
    motion.estimation = solution';

    %DisplaySM(scanK1.cart,motion.estimation',hscanSol);
    motion.estimation
    %pause;

    if (resMStep == 1)
        out = 1;
    elseif (resMStep == -1)
        out = -2;
    else
        nIteration = nIteration+1;
    end
end
hold off;

solution = motion.estimation'

out = 2;
nIteration
% Graphical representation
if (out > 0)
    DisplaySM(scanK1.cart,solution,hscanSol);
else
    disp('Unable to display results');
end