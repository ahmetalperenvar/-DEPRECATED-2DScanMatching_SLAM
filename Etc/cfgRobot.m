%Robot configuration

global MotionUncertainty;
global Opt;


Robot{1} = struct(...                     % DEFINED TRAJECTORY
  'id',                 1,...           % robot identifier
  'name',               'Dala',...      % robot name
  'type',               'atrv',...      % type of robot
  'motion',             'trajectory',...  % motion model (trajectory, realdata)
  'data', Opt.map.realdata,...          % file containing information
  'trajectory', trajr,...               % ground truth trajectory if available
  'odoTrajectory', [],...               % odometry trajectory if available
  'dxStd',           0.1*[1;1],...   % odo linear  and angular error std
  'errtype', 'gaussian',...             % added motion noise type
  'doStd',0.0001);                  % orientation error [roll pitch yaw]

MotionUncertainty = [ 0.1 0.1 0.00 ];