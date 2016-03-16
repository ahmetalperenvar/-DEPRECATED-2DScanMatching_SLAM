%Sensor Configuration
% 
%  Sensor{1} = struct(...
%   'id',                 1,...           % sensor identifier
%   'name',               'Sonar 2D',...  % sensor name
%   'type',               'Singlebeam2D',...   % type of sensor
%   'robot',              1,...           % robot where it is mounted
%   'position',           [0;0;0],...    % position in robot
%   'orientationDegrees', [0;0;0],... % orientation in robot, roll pitch yaw
%   'positionStd',        [0;0;0],...     % position error std
%   'beamStd',  [0.0524; 0.0524; 0.0524],...
%   'orientationStd',     [0;0;0],...     % orient. error std
%   'nBeams', 1,...   %image size
%   'maxWidth', 120,...  %multibeam angle coverage
%   'beamWidth', deg2rad(3),...   %single beam width
%   'minRange', 0.5,...  %beam strength in meters
%   'maxRange', 100,...
%   'dataPerScan', 1,... %data points returned each scan
%   'frameInMap', false);         % Angular error on each beam

% 

% %MULTIBEAM
% Sensor{1} = struct(...
%   'id',                 1,...           % sensor identifier
%   'name',               'Imagenex Delta T',...  % sensor name
%   'type',               'Multibeam',...   % type of sensor
%   'robot',              1,...           % robot where it is mounted
%   'position',           [0;0;0],...    % position in robot
%   'orientationDegrees', [0;1.5708;0],... % orientation in robot, roll pitch yaw
%   'positionStd',        [0;0;0],...     % position error std
%   'beamStd', [0; 0; 0],... % [0.0524; 0.0524; 0.0524],...
%   'orientationStd',     [0;0;0],...     % orient. error std
%   'nBeams', 120,...   %image size
%   'maxWidth', 120,...  %multibeam angle coverage
%   'beamWidth', 3,...   %single beam width
%   'minRange', 0.5,...  %beam strength in meters
%   'maxRange', 100,... 
%   'dataPerScan', 120,... %data points returned each scan
%   'frameInMap', false,...         % Angular error on each beam
%   'outliers', 0.001); 

global Opt;

%SINGLEBEAM 2D
SensorMultibeam = struct(...
  'id',                 1,...           % sensor identifier
  'name',               'Sonar 2D',...  % sensor name
  'type',               'Multibeam2D',...   % type of sensor
  'robot',              1,...           % robot where it is mounted
  'position',           [0;0;0],...    % position in robot
  'orientationDegrees', [0;0;0],... % orientation in robot, roll pitch yaw
  'positionStd',        [0;0;0],...     % position error std
  'beamStd', 1*[0.00; 0.000000; 0.000000],... % [0.0524; 0.0524; 0.0524],...
  'orientationStd',     [0;0;0],...     % orient. error std
  'nBeams', 360,...   %image size
  'maxWidth', 360,...  %multibeam angle coverage
  'beamWidth', deg2rad(3),...   %single beam width
  'minRange', 0.3,...  %beam strength in meters
  'maxRange', 100,... 
  'dataPerScan', 360,... %data points returned each scan
  'frameInMap', false,...         % Angular error on each beam
  'outliers', 0.000);           %Outliers probability 

% %REALDATA
SensorReal = struct(...
  'id',                 1,...           % sensor identifier
  'name',               'Laser',...  % sensor name
  'type',               'RealData',...   % type of sensor
  'robot',              1,...           % robot where it is mounted
  'position',           [0;0;0],...    % position in robot
  'orientationDegrees', [0;0;0],... % orientation in robot, roll pitch yaw
  'positionStd',        [0;0;0],...     % position error std
  'beamStd', 1*[0.09; 0.000005; 0.000005],... % [0.0524; 0.0524; 0.0524],...
  'orientationStd',     [0;0;0],...     % orient. error std
  'nBeams', 360,...   %image size
  'maxWidth', 360,...  %multibeam angle coverage
  'beamWidth', deg2rad(3),...   %single beam width
  'minRange', 0.5,...  %beam strength in meters
  'maxRange', 300,... 
  'dataPerScan', 360,... %data points returned each scan
  'frameInMap', false,...         % Angular error on each beam
  'outliers', 0.000,...
  'data', Opt.map.realdata); % file containing information)

Sensor{1} = SensorMultibeam;
