function [Rob,CRob] = createSlamStructures(Robot,Sen,Opt,Time)

% CREATESLAMSTRUCTURES  Initialize SLAM data structures from user data.

% Create robots and controls
Rob = createRobots(Robot,Opt);
CRob = createRobots(Robot,Opt);

Rob.Map = createMap(Rob,Sen,Opt,Time);
CRob.Map = createMap(Rob,Sen,Opt,Time);

