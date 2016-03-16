function Rob = createRobots(Robot,Tim)

% CREATEROBOTS Create robots structure array.
%   Rob = CREATEROBOTS(Robot) Generates the Rob structure that contains all
%   the information about the robot and the state vector and uncertainty.

global MotionUncertainty;

for rob = 1:numel(Robot)
    
    Ri = Robot{rob}; % input robot structure
    
    % identification
    Ro.rob     = rob;
    Ro.id      = Ri.id;
    Ro.name    = Ri.name;
    Ro.type    = Ri.type;
    Ro.motion  = Ri.motion;
    
    Ro.state0 = [Ri.trajectory(1:3,1); Ri.trajectory(4:6,1) ];
    
    %Ro.raw.globalCart2D = zeros(Opt.scan.maxscanpoints,3);
    %Ro.raw.globalPolar2D = zeros(Opt.scan.maxscanpoints,3);
    Ro.raw.localCart = []; %zeros(Opt.scan.maxscanpoints,3);
    Ro.raw.localPolar = []; %zeros(Opt.scan.maxscanpoints,2);
    Ro.raw.covm = [];
    
    %Ro.filtered.globalCart2D = zeros(Opt.scan.maxscanpoints,3);
    %Ro.filtered.globalPolar2D = zeros(Opt.scan.maxscanpoints,3);
    Ro.filtered.localCart = []; %zeros(Opt.scan.maxscanpoints,3);
    Ro.filtered.localPolar = []; %zeros(Opt.scan.maxscanpoints,2);
    Ro.filtered.covm = [];
    Ro.filtered.last = 0;
    Ro.filtered.pos = [0 0 0]';
    Ro.filtered.poses = [];
    Ro.filtered.timePoses = [];
    
    Ro.Map = [];
    Ro.lastcorrection = 0;
    
    Ro.error_type = Ri.errtype;
    
    % control
    switch Ri.motion

        case {'trajectory'}
            
            Ro.con.u    = [0 0 0];
            Ro.con.incr = 1;
            Ro.con.U    = diag([Ri.dxStd; Ri.doStd]);
            Ro.con.d = [0 0 0];
            Ro.vel.x    = [];
            Ro.vel.P    = [];
            Ro.trajectory = Ri.trajectory;
            
        otherwise
            error('Unknown motion model %s from robot %d.',Robot.motion,Robot.id);
    end
    
    Ro.sensors = [];
    
    % Robot frame
    ep = [0; 0; 0];
    EP = Ro.con.U;
    
    %[qp,QP] = propagateUncertainty(ep,EP,@epose2qpose); % frame and cov. in quaternion
    
    % Define the 4D state and Uncertainty
    Ro.state.x  = ep;   %SLAM State and Covariance
    Ro.state.P  = EP;
    
    Ro.state.dr  = ep;  % Dead Reckoning
    Ro.state.gt = ep;   %Ground Truth
    
    Ro.state.P_Added = Ro.state.P;
    
    
    
    steps = ceil((Tim.lastFrame - Tim.firstFrame) / Tim.step);
    
    Ro.state.x_full = [];% zeros(size(Ro.state.x,1), steps);
    Ro.state.dr_full = zeros(3,steps);
    Ro.state.gt_full = zeros(3,steps);
    Ro.state.P_full = []; %zeros(size(Ro.state.x,1),size(Ro.state.x,1), steps);
    
    
    Rob(rob) = Ro; % output robot structure
    
end




% ========== End of function - Start GPL license ==========


%   # START GPL LICENSE

%---------------------------------------------------------------------
%
%   This file is part of SLAMTB, a SLAM toolbox for Matlab.
%
%   SLAMTB is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   SLAMTB is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>.
%
%---------------------------------------------------------------------

%   SLAMTB is Copyright 2007,2008,2009
%   by Joan Sola, David Marquez and Jean Marie Codol @ LAAS-CNRS.
%   See on top of this file for its particular copyright.

%   # END GPL LICENSE

