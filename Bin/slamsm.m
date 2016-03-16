%% I Initialize all data structures

% Init data
[Rob,SimSen,SimWorld,Tim] = createSimStructures(...
    Robot,...
    Sensor,...      % all user data
    RealWorld,...
    Time);

% Create empty Raw structure, it stores unprocessed measurements
Raw.type = '';
Raw.data = struct([]);

if Opt.error.display_result
    createPlot();
end

NI= [];

%% II Init data logging
% TODO: Create source and/or destination files and paths for data input and
% logs.
% TODO: do something here to collect data for post-processing or
% plotting. Think about collecting data in files using fopen, fwrite,
% etc., instead of creating large Matlab variables for data logging.

% Clear user data - not needed anymore


nIter = 0;
global DISPLACEMENT currentFrame LAST_SCANS;
%% III Main loop
for currentFrame = Tim.firstFrame : Tim.step : Tim.lastFrame
    
    % 1. SIMULATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulate robots
    nIter = nIter+1;
    %Tim.prevFrame = Tim.currentFrame-1;
    Tim.currentFrame = currentFrame;
    
    %% Odometry Reading
    
    
    % Simulate Motion and save the Ground Truth when available
    Rob = simMotion(Rob,Tim,1);
    odocart = (Rob.con.u(1:2))';
    % Filter Update
    
    % Update the Dead Reckoning Estimation. DO NOT UPDATE THE MAP AFTER
    Rob.state.dr = odo2_cart(Rob.state.dr,Rob.con.u);
    Rob.state.dr(3) = normAngle(Rob.state.dr(3));
        
    
    if Opt.filter.usefilter
        if currentFrame>1
            [Rob] = Cloning(Rob,3);
        end
        [Rob] = Prediction_ScanMatching_v1(Rob);
        Rob.state.x_full = reshape(Rob.state.x,3,size(Rob.state.x,1)/3);
        Rob.state.dr_full(:,Tim.currentFrame) = Rob.state.dr;
        Rob.state.gt_full(:,Tim.currentFrame) = Rob.state.gt;
    else
        % If no filter is used, the estimation is simple dead reckoning
        Rob.state.x = odo2_cart(Rob.state.x,Rob.con.u);
        Rob.state.x(3) = normAngle(Rob.state.x(3));
        Rob = addPoseToMap(Rob,Tim);
    end
 
    %DEBUG
    DISPLACEMENT_EU = ( Rob.state.d )
    DISPLACEMENT_AMOUNT = sqrt(Rob.state.d(1)^2 +Rob.state.d(2)^2)
    
    %% Observation Reading
    
    %Observe  data (noise is added)
    [RawL, SimSen] = simObservation(Rob, SimSen, SimWorld);
    
    SimSen.raw.timed(currentFrame).localCart = RawL.data.localCart;
    SimSen.raw.timed(currentFrame).localPolar = RawL.data.localPolar;
    
    SimSen.raw.localCart = [ SimSen.raw.localCart; RawL.data.localCart];
    SimSen.raw.localPolar = [ SimSen.raw.localPolar; RawL.data.localPolar];
    %Rob.raw.covm = cat(3, Rob.raw.covm, RawL.data.covm(:,:,:) );
    
    
    % Filter the scan readings
    if size(SimSen.raw.localCart,1) >= Opt.scan.maxscanpoints
        Rob = filterScan(Rob, SimSen, Tim,Opt);
        SimSen.raw.localCart = [];
        SimSen.raw.localPolar = [];
    end
    
    
    %% Scanmatching
    % retrieve a new map S and its applied displacement R
    % and t DEBUG VERSION
    if performSM(Rob,Tim,Opt)
        
        RefScan = getScan(Rob.Map,nIter-1);
        
        % Correction is performed only if points are present
        if ~isempty(RefScan) && size(RefScan.data.localCart,1) > 1 &&...
                size(Rob.filtered.localCart,1) > 1

            if Opt.filter.usefilter
            
            N = size(Rob.state.x,1);
            H = zeros(3,N);
            
            %find test scan position
            j = currentFrame-1;
            elm_x = N -(3*j-1); elm_y = N -(3*j-2); elm_th = N -(3*j-3);
            testX = [Rob.state.x(elm_x); Rob.state.x(elm_y); Rob.state.x(elm_th)];
            
            %Predict displacement
            testXinv = invert(testX); %do the inversion transformation
            res = compound(testXinv.x,Rob.state.x(1:3)); %do the compounding
            motion.q = res.x;
            
            %Build H matrix
            H(1:3,1:3) = res.Jb;
            H(1:3,elm_x:elm_th) = (res.Ja * testXinv.oJ);
            
            %Predicted covariance
            motion.Pq = H*Rob.state.P*H';
            motion.H = H;
            
            
            Rob.con.u(1:2)=res.x(1:2);
            Rob.con.Pq = motion.Pq;
            
            else
                                u_inv = frameInv(Rob.state.x_full(: , currentFrame-1));
                u_r = frameRef(Rob.state.x,u_inv);
                Rob.con.u(1:2)=u_r(1:2);
                
            end
            
            [R t NI] = Opt.scanmatcher.handle(RefScan.data, Rob.filtered, Rob);

            R
            t
            
            if Opt.filter.usefilter
                [Rob.state.x,Rob.state.P] = Correction_ScanMatching([t R]',motion, Rob.state.x,Rob.state.P);
            else
                Rob = correctPose(Rob,R,t);
                Rob = addPoseToMap(Rob,Tim);
            end
            
             Rob.lastcorrectionP = [t R'];
        end
        
        Rob.Map.x = Rob.state.x;
        [Rob.Map Rob.filtered] = addPointsToMap(Rob.filtered,Rob.Map,Tim,Rob);
        Rob.lastcorrection = Tim.currentFrame;

    else
        [Rob.Map Rob.filtered] = addPointsToMap(Rob.filtered,Rob.Map,Tim,Rob);
    end

    %% PLOTTING
    
    if (size(Rob.Map.grid,1) > 0) && Opt.error.display_result == 1
        
        if Opt.plot.points_cr
            set(Opt.plot.points_cr_h,'XData',Rob.Map.grid(:,1),'YData',Rob.Map.grid(:,2));
        end
        
        if Opt.plot.ground_truth
            Opt.plot.ground_truth_data = [Opt.plot.ground_truth_data; Rob.state.gt(:,end)' ];
            set(Opt.plot.ground_truth_h,'XData',Opt.plot.ground_truth_data(:,1),'YData',Opt.plot.ground_truth_data(:,2));
            grob = graphicsRobot(Opt.plot.ground_truth_data(end,:)');
            set(Opt.plot.ground_truth_h_rob,'XData',grob(1,:),'YData',grob(2,:));
        end
        
        if Opt.plot.dead_reckoning
            Opt.plot.dead_reckoning_data = [Opt.plot.dead_reckoning_data ; frameRef(Rob.state.dr(:,end),Rob.state0([1 2 6]),0)'];
            set(Opt.plot.dead_reckoning_h,'XData',Opt.plot.dead_reckoning_data(:,1),'YData',Opt.plot.dead_reckoning_data(:,2));
            grob = graphicsRobot(Opt.plot.dead_reckoning_data(end,:)');
            set(Opt.plot.dead_reckoning_h_rob,'XData',grob(1,:),'YData',grob(2,:));
        end
        
        if Opt.plot.corrected
            Opt.plot.corrected_data = [Opt.plot.corrected_data; frameRef(Rob.state.x(:,end),Rob.state0([1 2 6]),0)'];
            set(Opt.plot.corrected_h,'XData',Opt.plot.corrected_data(:,1),'YData',Opt.plot.corrected_data(:,2));
            grob = graphicsRobot(Opt.plot.corrected_data(end,:)');
            set(Opt.plot.corrected_h_rob,'XData',grob(1,:),'YData',grob(2,:));
            
            if Opt.plot.correction_uncertainty
                draw_ellipse(Opt.plot.corrected_data(end,:),Rob.state.P,'r',Opt.plot.correction_uncertainty_h);
            end
        end
        
        drawnow;
    end
    out = 1;
end


if ~isempty(NI)
    error_gt =  errorReport(Rob,SimSen,Opt, NI);
    save('std_last','error_gt');
end
