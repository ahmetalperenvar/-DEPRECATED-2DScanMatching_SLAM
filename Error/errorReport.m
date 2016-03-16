function [ err_mat ] = errorReport( Rob, Sen, Opt, NI )
%ERRORREPORT Generate a report on the error using the ground truth

%Find the angular shift in the yaw using quaternion mathematics

Map = Rob.Map;

ID = Opt.random.seed;
sm_iterations = Opt.scanmatcher.iterations;
sm_br = Opt.scanmatcher.Br;



t1 = Rob.state.gt;
t2 = Rob.state.x
q1 = Rob.state.x(3);
q2 = Rob.state.gt(3);

err_q = normAngle(q1 - q2)
err_x = t1(1) - t2(1)
err_y = t1(2) - t2(2)

err_mat = [ err_x err_y 0;
            [0 0 err_q] ;
            Rob.state.d(1:2)' 0;
            [0 0 Rob.state.d(3)];
            Rob.lastcorrectionP(1:2) 0;
            [0 0 Rob.lastcorrectionP(3)];
            NI NI NI];

terr_x = 0;
terr_y = 0;
terr_z = 0;
terr_r = 0;
terr_p = 0;
terr_yaw = 0;

out = 1;

% fid = fopen('exp.txt', 'a');
% fprintf(fid, '\n\n Scan SEED: %6.4f \n\n', ID);
% 
% fprintf(fid, 'Parameters of the computation' );
% fprintf(fid, 'Linear Error (Odometry): %6.4f %6.4f %6.4f \n', errRob(1:3));
% fprintf(fid, 'Angular Error (Odometry): %6.4f %6.4f %6.4f \n', errRob(4:6));
% fprintf(fid, 'Orientation Error Robot: %6.4f %6.4f %6.4f \n', errRob(7:9));
% fprintf(fid, 'Error Sensor: %6.4f %6.4f %6.4f \n', errSen');
% fprintf(fid, 'ScanMatcher Type: %s \n', sm_type);
% fprintf(fid, 'ScanMatcher Max Angle: %6.4f \n', sm_br);
% fprintf(fid, 'ScanMatcher Iterations: %6.4f \n', sm_iterations');
% 
% fprintf(fid, 'Corrected Robot Pose \n' );
% 
% fprintf(fid, 'Error X: %6.4f \n', err_x);
% fprintf(fid, 'Error Y: %6.4f \n', err_y);
% fprintf(fid, 'Error Z: %6.4f \n', err_z);
% fprintf(fid, 'Error Angle_Roll: %6.4f \n', err_q(1) );
% fprintf(fid, 'Error Angle_Pitch: %6.4f \n', err_q(2) );
% fprintf(fid, 'Error Angle_Yaw: %6.4f \n', err_q(3) );
% 
% fclose(fid);

% sf = ['mean_last'];
% 
% if exist( [sf '.mat'])
%     load(sf,'terr*');
% end
% terr_x = terr_x + err_x;
% terr_y = terr_y + err_y;
% terr_z = terr_z + err_z;
% terr_r = terr_r + err_q(1);
% terr_p = terr_p + err_q(2);
% terr_yaw = terr_yaw + err_q(3);
% 
% save(sf,'terr*');

end

