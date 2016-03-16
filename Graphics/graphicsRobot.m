function [ trpt ] = graphicsRobot( pos)
%DISPLAYROBOT Display robot shape

global Opt
scale = Opt.plot.robot_scale;

q = pos(3);
pos = [pos(1:2); 1];
trp = [ -1 -0.5 1; 1 0 1; -1 0.5 1];
trp = (trp) * scale;

trpt = [ e2R([0 0 q]) * trp(1,:)' + pos  ...
    e2R([0 0 q]) * trp(2,:)' + pos  ...
    e2R([0 0 q]) * trp(3,:)' + pos ];

end

