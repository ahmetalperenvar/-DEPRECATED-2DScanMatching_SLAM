function [Rob,SimSen,SimW,Tim] = createSimStructures(Robot,Sensor,World,Time)

% CREATESIMSTRUCTURES Create simulation data structures.

global Opt

Tim = createTime(Time);

% Create robots and controls
Rob = createRobots(Robot,Tim);          % Ground Truth


if Opt.filter.usefilter 
   Rob = KalmanFilter_init(Rob);
end

% Create sensors
SimSen = createSensors(Sensor);  % Range Finder

% Create world
SimW = createWorld(World);

% Create time variables


% Create Map Structs

Rob.Map = createMap(Rob,SimSen,Tim);


